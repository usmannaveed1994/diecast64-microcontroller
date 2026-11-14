// Helpers for the walkie-talkie channel in app/components/walkieTalkie.tsx.
//
// The car and the browser exchange raw 16 bit mono PCM at 16 kHz over MQTT (see
// the "WALKIE AUDIO STREAM" section of firmware/microcontroller/race_car_main.ino).
// Nothing is compressed: the ESP32 copies samples straight out of the INMP441
// and into the MAX98357A, and everything here converts between that wire format
// and the Float32 buffers the Web Audio API works with.

/** Must match AUDIO_STREAM_SAMPLE_RATE in the firmware. */
export const WALKIE_SAMPLE_RATE = 16000;

/** Bytes per published packet, i.e. AUDIO_CHUNK_BYTES in the firmware. */
export const WALKIE_CHUNK_BYTES = 1024;

/** Samples per published packet: 512 samples, 32 ms of audio at 16 kHz. */
export const WALKIE_CHUNK_SAMPLES = WALKIE_CHUNK_BYTES / 2;

/** Longest transmission both ends agree to send, in milliseconds. */
export const WALKIE_MAX_RECORD_MS = 5000;

/**
 * Interpolation state for {@link resampleToWalkie}. It is kept between buffers so
 * a buffer boundary does not turn into a click in the published stream.
 */
export type ResamplerState = {
  /** Input samples consumed per output sample (hardware rate / 16 kHz). */
  ratio: number;
  /** Position inside the next input buffer that the resampler still owes us. */
  phase: number;
};

/** Builds the state needed to convert `inputRate` audio down to 16 kHz. */
export function createResampler(inputRate: number): ResamplerState {
  const ratio = inputRate > 0 ? inputRate / WALKIE_SAMPLE_RATE : 1;

  return {
    ratio: Math.max(1, ratio),
    phase: 0,
  };
}

/** Converts Float32 samples (-1..1) into the 16 bit PCM the car expects. */
export function floatToInt16(input: Float32Array): Int16Array {
  const output = new Int16Array(input.length);

  for (let i = 0; i < input.length; i++) {
    output[i] = toInt16(input[i]);
  }

  return output;
}

/** Converts 16 bit PCM from the car into Float32 samples for playback. */
export function int16ToFloat32(input: Int16Array): Float32Array {
  const output = new Float32Array(input.length);

  for (let i = 0; i < input.length; i++) {
    // -32768 can only reach -1, so negative values scale by 2^15 and positive
    // ones by 2^15 - 1. Same convention as floatToInt16().
    output[i] = input[i] < 0 ? input[i] / 0x8000 : input[i] / 0x7fff;
  }

  return output;
}

/**
 * Decimates a captured buffer to 16 kHz with linear interpolation.
 *
 * The microphone runs at whatever rate the audio device picked (44.1 or 48 kHz
 * on most phones), so the samples have to be squeezed before they are published
 * or the car would play them back ~3x too slow and an octave low.
 */
export function resampleToWalkie(input: Float32Array, state: ResamplerState): Int16Array {
  if (input.length === 0) return new Int16Array(0);

  // Already at 16 kHz (Chrome honours the AudioContext sampleRate): nothing to do.
  if (state.ratio <= 1.0001) {
    return floatToInt16(input);
  }

  const output = new Int16Array(Math.ceil(input.length / state.ratio) + 1);
  let written = 0;
  let position = state.phase;

  while (position < input.length) {
    const index = Math.floor(position);
    const fraction = position - index;

    const first = input[index];
    const second = index + 1 < input.length ? input[index + 1] : first;

    output[written++] = toInt16(first + (second - first) * fraction);
    position += state.ratio;
  }

  // Carry the leftover position into the next buffer to keep the cadence steady.
  state.phase = position - input.length;

  return output.subarray(0, written);
}

/**
 * 0..1 level for the meters, using the same -60 dBFS .. 0 dBFS scale as the
 * firmware's serial level bar so both ends read the same.
 */
export function audioLevel(samples: Int16Array): number {
  if (samples.length === 0) return 0;

  let sumSquares = 0;

  for (let i = 0; i < samples.length; i++) {
    const normalized = samples[i] / 0x8000;
    sumSquares += normalized * normalized;
  }

  const rms = Math.sqrt(sumSquares / samples.length);
  if (rms <= 0) return 0;

  const dbfs = 20 * Math.log10(rms);
  const level = (dbfs + 60) / 60;

  return Math.min(1, Math.max(0, level));
}

/**
 * Wraps received PCM packets in a canonical 44 byte WAV header so a clip can be
 * replayed with an <audio> element and downloaded.
 */
export function encodeWav(chunks: Int16Array[], sampleRate = WALKIE_SAMPLE_RATE): Blob {
  const totalSamples = chunks.reduce((total, chunk) => total + chunk.length, 0);
  const dataBytes = totalSamples * 2;

  const buffer = new ArrayBuffer(44 + dataBytes);
  const view = new DataView(buffer);

  writeAscii(view, 0, 'RIFF');
  view.setUint32(4, 36 + dataBytes, true); // file size minus the first 8 bytes
  writeAscii(view, 8, 'WAVE');

  writeAscii(view, 12, 'fmt ');
  view.setUint32(16, 16, true); // size of the fmt chunk
  view.setUint16(20, 1, true); // format: PCM
  view.setUint16(22, 1, true); // channels: mono
  view.setUint32(24, sampleRate, true);
  view.setUint32(28, sampleRate * 2, true); // byte rate = rate * block align
  view.setUint16(32, 2, true); // block align = channels * bytes per sample
  view.setUint16(34, 16, true); // bits per sample

  writeAscii(view, 36, 'data');
  view.setUint32(40, dataBytes, true);

  let offset = 44;

  for (const chunk of chunks) {
    for (let i = 0; i < chunk.length; i++) {
      view.setInt16(offset, chunk[i], true);
      offset += 2;
    }
  }

  return new Blob([buffer], { type: 'audio/wav' });
}

/** "3.4 s" for the transmission list. */
export function formatDuration(durationMs: number): string {
  return `${(durationMs / 1000).toFixed(1)} s`;
}

function toInt16(value: number): number {
  const scaled = value < 0 ? value * 0x8000 : value * 0x7fff;

  if (scaled > 32767) return 32767;
  if (scaled < -32768) return -32768;

  return Math.round(scaled);
}

function writeAscii(view: DataView, offset: number, text: string): void {
  for (let i = 0; i < text.length; i++) {
    view.setUint8(offset + i, text.charCodeAt(i));
  }
}
