'use client';

import { useCallback, useEffect, useRef, useState } from 'react';
import type {
  KeyboardEvent as ReactKeyboardEvent,
  PointerEvent as ReactPointerEvent,
} from 'react';
import mqtt, { MqttClient } from 'mqtt';
import {
  AudioLines,
  CircleStop,
  Download,
  Headphones,
  Loader2,
  Mic,
  Radio,
  Volume2,
  WifiOff,
} from 'lucide-react';
import { Alert, AlertAction, AlertDescription, AlertTitle } from '@/components/ui/alert';
import { Badge } from '@/components/ui/badge';
import { Button } from '@/components/ui/button';
import {
  Card,
  CardAction,
  CardContent,
  CardDescription,
  CardFooter,
  CardHeader,
  CardTitle,
} from '@/components/ui/card';
import { cn } from '@/lib/utils';
import {
  WALKIE_CHUNK_BYTES,
  WALKIE_CHUNK_SAMPLES,
  WALKIE_MAX_RECORD_MS,
  WALKIE_SAMPLE_RATE,
  audioLevel,
  createResampler,
  encodeWav,
  formatDuration,
  int16ToFloat32,
  resampleToWalkie,
  type ResamplerState,
} from '@/lib/audio';

const BROKER_URL = 'wss://broker.hivemq.com:8884/mqtt';
const DEVICE_ROOT = 'usman-esp32-9988';
const AUDIO_TOPIC_ROOT = `${DEVICE_ROOT}/audio`;
// Car -> browser (the INMP441) and browser -> car (the MAX98357A), see the
// "WALKIE AUDIO STREAM" section of firmware/microcontroller/race_car_main.ino.
const MIC_TOPIC_ROOT = `${AUDIO_TOPIC_ROOT}/mic`;
const SPEAKER_TOPIC_ROOT = `${AUDIO_TOPIC_ROOT}/speaker`;

// Every microphone packet carries a 2 byte sequence number before the PCM.
const AUDIO_HEADER_BYTES = 2;

// mqtt.js reports `offline`/`error` for any dropped socket, including a single
// failed handshake. Wait a moment before surfacing the destructive alert so a
// quick reconnect does not flash "Broker unreachable".
const OFFLINE_GRACE_MS = 2000;

// Live playback: each packet is scheduled right after the previous one, with a
// small lead so the Web Audio clock has time to line it up.
const PLAYBACK_LEAD_S = 0.15;
const PLAYBACK_MAX_LAG_S = 1.5;

// The device falls silent without sending "end" when the socket drops mid-clip.
const RX_STALL_MS = 1200;
const RX_LEVEL_INTERVAL_MS = 80;
const TX_LEVEL_INTERVAL_MS = 80;
const TX_TIMER_INTERVAL_MS = 50;

// 2048 frames at 44.1 kHz is ~46 ms of audio per callback: small enough for a
// responsive level meter, large enough not to flood the main thread.
const CAPTURE_BUFFER_SIZE = 2048;

// Only the newest transmissions stay on screen; each one pins its decoded audio
// in memory as an object URL.
const MAX_CLIPS = 5;
const LEVEL_SEGMENTS = 24;

type ConnectionStatus = 'connecting' | 'connected' | 'offline';
type TransmissionState = 'idle' | 'arming' | 'live';

type ReceivedClip = {
  key: string;
  receivedAt: number;
  durationMs: number;
  url: string;
};

/** Audio collected between the device's "start" and "end" messages. */
type Reception = {
  id: string;
  chunks: Int16Array[];
  samples: number;
  receivedAt: number;
  announcedDurationMs: number;
};

/**
 * mqtt.js only types `publish` for `string | Buffer`, but its browser build hands the
 * message straight to the socket (mqtt-packet accepts typed arrays, and the bundled
 * buffer polyfill measures them correctly). The cast keeps a separate `buffer`
 * dependency out of the bundle just to wrap four bytes per sample.
 */
type MqttPayload = Parameters<MqttClient['publish']>[1];

const STATUS_BADGE: Record<ConnectionStatus, { label: string; className: string }> = {
  connected: {
    label: 'Online',
    className: 'border-emerald-500/20 bg-emerald-500/10 text-emerald-600',
  },
  connecting: {
    label: 'Connecting',
    className: 'border-amber-500/20 bg-amber-500/10 text-amber-600',
  },
  offline: {
    label: 'Offline',
    className: 'border-destructive/20 bg-destructive/10 text-destructive',
  },
};
export default function WalkieTalkie() {
  const clientRef = useRef<MqttClient | null>(null);

  const audioContextRef = useRef<AudioContext | null>(null);
  const micStreamRef = useRef<MediaStream | null>(null);
  const micSourceRef = useRef<MediaStreamAudioSourceNode | null>(null);
  const processorRef = useRef<ScriptProcessorNode | null>(null);
  const sinkRef = useRef<GainNode | null>(null);

  // Transmission state read by the audio callback and the timer, so it does not
  // have to wait for a re-render to be up to date.
  const txStateRef = useRef<TransmissionState>('idle');
  const pressHeldRef = useRef(false);
  const capturingRef = useRef(false);
  const txStartedAtRef = useRef(0);
  const txTimerRef = useRef<ReturnType<typeof setInterval> | null>(null);
  const txLevelAtRef = useRef(0);
  const resamplerRef = useRef<ResamplerState>(createResampler(WALKIE_SAMPLE_RATE));
  const chunkRef = useRef<{ data: Int16Array; fill: number } | null>(null);

  // Reception state shared with the MQTT message handlers.
  const receptionRef = useRef<Reception | null>(null);
  const clipsRef = useRef<ReceivedClip[]>([]);
  const playheadRef = useRef(0);
  const rxLevelAtRef = useRef(0);

  const [connectionStatus, setConnectionStatus] = useState<ConnectionStatus>('connecting');
  const [audioBlocked, setAudioBlocked] = useState(true);
  const [rxActive, setRxActive] = useState(false);
  const [rxLevel, setRxLevel] = useState(0);
  const [clips, setClips] = useState<ReceivedClip[]>([]);
  const [txState, setTxState] = useState<TransmissionState>('idle');
  const [txElapsedMs, setTxElapsedMs] = useState(0);
  const [txLevel, setTxLevel] = useState(0);
  const [error, setError] = useState<string | null>(null);

  /**
   * Creates (once) and resumes the AudioContext. Browsers only let audio start
   * inside a user gesture, so this is called from the first pointer/key event and
   * again from the push-to-talk button.
   */
  const ensureAudioContext = useCallback(async () => {
    let context = audioContextRef.current;

    if (!context) {
      try {
        // Asking for 16 kHz lets Chrome resample the capture for us. Safari
        // ignores the option and keeps the hardware rate, which is why the
        // recorder resamples to WALKIE_SAMPLE_RATE on its own.
        context = new AudioContext({ sampleRate: WALKIE_SAMPLE_RATE });
      } catch {
        context = new AudioContext();
      }

      const created = context;
      created.onstatechange = () => setAudioBlocked(created.state !== 'running');
      audioContextRef.current = created;
    }

    if (context.state === 'suspended') {
      await context.resume();

      // iOS unlocks the output only once a buffer has been started inside a
      // gesture, so play one silent sample.
      const buffer = context.createBuffer(1, 1, context.sampleRate);
      const source = context.createBufferSource();
      source.buffer = buffer;
      source.connect(context.destination);
      source.start(0);
    }

    setAudioBlocked(context.state !== 'running');

    return context;
  }, []);

  const releaseMicrophone = useCallback(() => {
    capturingRef.current = false;

    processorRef.current?.disconnect();
    processorRef.current = null;

    micSourceRef.current?.disconnect();
    micSourceRef.current = null;

    sinkRef.current?.disconnect();
    sinkRef.current = null;

    micStreamRef.current?.getTracks().forEach((track) => track.stop());
    micStreamRef.current = null;
  }, []);

  /** Publishes one 1 kB PCM packet to the car's speaker. */
  const publishChunk = useCallback((samples: Int16Array) => {
    const client = clientRef.current;
    if (!client || !client.connected || samples.length === 0) return;

    // Little endian 16 bit PCM, exactly what writeSpeakerChunk() on the ESP32
    // expects. A fresh array per packet keeps mqtt.js from holding on to a buffer
    // that the next capture callback rewrites.
    const frame = new Uint8Array(samples.length * 2);
    const view = new DataView(frame.buffer);

    for (let i = 0; i < samples.length; i++) {
      view.setInt16(i * 2, samples[i], true);
    }

    client.publish(`${SPEAKER_TOPIC_ROOT}/chunk`, frame as MqttPayload, { qos: 0 });
  }, []);

  /**
   * Opens the microphone once and keeps the capture graph alive for the whole
   * session: on iOS a stopped stream can only be reopened from another gesture,
   * which would make every press ask for the device again.
   */
  const ensureMicrophone = useCallback(
    async (context: AudioContext) => {
      if (processorRef.current) return;

      if (!navigator.mediaDevices?.getUserMedia) {
        throw new Error('This browser cannot capture audio');
      }

      const stream = await navigator.mediaDevices.getUserMedia({
        audio: {
          channelCount: 1,
          echoCancellation: true,
          noiseSuppression: true,
          autoGainControl: true,
        },
      });

      const source = context.createMediaStreamSource(stream);

      // ScriptProcessor is deprecated, but it is the only capture node that works
      // everywhere (iOS included) without shipping a separate AudioWorklet module.
      const processor = context.createScriptProcessor(CAPTURE_BUFFER_SIZE, 1, 1);

      // A processor only runs while it is connected to the destination, so it is
      // routed through a muted gain node: without this the mic would be echoed
      // straight back to the phone's speaker.
      const sink = context.createGain();
      sink.gain.value = 0;

      processor.onaudioprocess = (event) => {
        if (!capturingRef.current) return;

        const samples = resampleToWalkie(
          event.inputBuffer.getChannelData(0),
          resamplerRef.current
        );

        const pending = chunkRef.current;
        if (!pending || samples.length === 0) return;

        for (let i = 0; i < samples.length; i++) {
          pending.data[pending.fill++] = samples[i];

          // 512 samples = one packet = 32 ms of audio.
          if (pending.fill === WALKIE_CHUNK_SAMPLES) {
            publishChunk(pending.data);
            pending.fill = 0;
          }
        }

        const now = performance.now();
        if (now - txLevelAtRef.current > TX_LEVEL_INTERVAL_MS) {
          txLevelAtRef.current = now;
          setTxLevel(audioLevel(samples));
        }
      };

      source.connect(processor);
      processor.connect(sink);
      sink.connect(context.destination);

      micStreamRef.current = stream;
      micSourceRef.current = source;
      processorRef.current = processor;
      sinkRef.current = sink;
    },
    [publishChunk]
  );

  /** Ends the current transmission, flushing the partial packet first. */
  const stopTransmission = useCallback(() => {
    if (!capturingRef.current) return;

    capturingRef.current = false;

    if (txTimerRef.current !== null) {
      clearInterval(txTimerRef.current);
      txTimerRef.current = null;
    }

    // Publish the tail of the last packet so the final syllable is not lost.
    const pending = chunkRef.current;
    if (pending && pending.fill > 0) {
      publishChunk(pending.data.subarray(0, pending.fill));
      pending.fill = 0;
    }

    const elapsedMs = Math.min(
      performance.now() - txStartedAtRef.current,
      WALKIE_MAX_RECORD_MS
    );

    setTxState('idle');
    txStateRef.current = 'idle';
    setTxLevel(0);
    setTxElapsedMs(elapsedMs);

    const client = clientRef.current;
    if (client?.connected) {
      client.publish(
        `${SPEAKER_TOPIC_ROOT}/end`,
        JSON.stringify({ durationMs: Math.round(elapsedMs) }),
        { qos: 0 }
      );
    }
  }, [publishChunk]);

  const startTransmission = useCallback(async () => {
    if (txStateRef.current !== 'idle') return;

    pressHeldRef.current = true;
    setError(null);
    setTxState('arming');
    txStateRef.current = 'arming';

    let context: AudioContext;

    try {
      context = await ensureAudioContext();
      await ensureMicrophone(context);
    } catch (cause) {
      console.error('Microphone unavailable:', cause);

      setError(
        'The microphone is unavailable. Allow it for this site in the browser settings and try again.'
      );
      setTxState('idle');
      txStateRef.current = 'idle';

      return;
    }

    const client = clientRef.current;
    const blocked = !pressHeldRef.current || !client?.connected;

    // Releasing the button while the permission prompt was open, or talking to a
    // broker that is still reconnecting, must not open a transmission nobody can
    // hear (and that the car would wait on for 1.5 s).
    if (blocked) {
      if (!client?.connected) setError('Not connected to the MQTT broker yet.');
      setTxState('idle');
      txStateRef.current = 'idle';

      return;
    }

    resamplerRef.current = createResampler(context.sampleRate);
    chunkRef.current = { data: new Int16Array(WALKIE_CHUNK_SAMPLES), fill: 0 };

    client.publish(
      `${SPEAKER_TOPIC_ROOT}/start`,
      JSON.stringify({ sampleRate: WALKIE_SAMPLE_RATE, channels: 1, bits: 16 }),
      { qos: 0 }
    );

    capturingRef.current = true;
    txStartedAtRef.current = performance.now();

    setTxState('live');
    txStateRef.current = 'live';
    setTxElapsedMs(0);

    txTimerRef.current = setInterval(() => {
      const elapsedMs = performance.now() - txStartedAtRef.current;
      setTxElapsedMs(Math.min(elapsedMs, WALKIE_MAX_RECORD_MS));

      // Same hard cap as the firmware, so the car never buffers more than it can
      // play back in a reasonable time.
      if (elapsedMs >= WALKIE_MAX_RECORD_MS) stopTransmission();
    }, TX_TIMER_INTERVAL_MS);
  }, [ensureAudioContext, ensureMicrophone, stopTransmission]);

  // MQTT session: subscribe to the car's microphone stream and play it back live.
  useEffect(() => {
    let disposed = false;
    let offlineTimer: ReturnType<typeof setTimeout> | null = null;
    let isOfflineReported = false;
    let stallTimer: ReturnType<typeof setTimeout> | null = null;

    const clearOfflineTimer = () => {
      if (offlineTimer !== null) {
        clearTimeout(offlineTimer);
        offlineTimer = null;
      }
    };

    const reportOffline = () => {
      if (disposed || isOfflineReported) return;

      isOfflineReported = true;
      clearOfflineTimer();
      setConnectionStatus('offline');
    };

    const pushClip = (clip: ReceivedClip) => {
      const next = [clip, ...clipsRef.current];

      // Only the newest clips stay on screen: each one pins its decoded audio in
      // memory until the object URL is revoked.
      for (const dropped of next.splice(MAX_CLIPS)) {
        URL.revokeObjectURL(dropped.url);
      }

      clipsRef.current = next;
      setClips(next);
    };

    /** Closes the reception in progress and turns it into a replayable clip. */
    const finishReception = () => {
      if (stallTimer !== null) {
        clearTimeout(stallTimer);
        stallTimer = null;
      }

      const reception = receptionRef.current;
      receptionRef.current = null;

      setRxActive(false);
      setRxLevel(0);

      if (!reception || reception.chunks.length === 0) return;

      const durationMs =
        reception.announcedDurationMs > 0
          ? reception.announcedDurationMs
          : Math.round((reception.samples / WALKIE_SAMPLE_RATE) * 1000);

      pushClip({
        key: `${reception.id}-${reception.receivedAt}`,
        receivedAt: reception.receivedAt,
        durationMs,
        url: URL.createObjectURL(encodeWav(reception.chunks)),
      });
    };

    const scheduleLivePlayback = (samples: Int16Array) => {
      const context = audioContextRef.current;

      // Nothing to do until the browser has let audio start (see the unlock
      // effect below): the clip still lands in the list for later.
      if (!context || context.state !== 'running') return;

      const buffer = context.createBuffer(1, samples.length, WALKIE_SAMPLE_RATE);
      buffer.getChannelData(0).set(int16ToFloat32(samples));

      const source = context.createBufferSource();
      source.buffer = buffer;
      source.connect(context.destination);

      // Packets are chained back to back so a transmission plays seamlessly. If
      // the queue drifted (throttled tab, broker hiccup) it is re-anchored with a
      // short lead instead of stacking latency on top of latency.
      const now = context.currentTime;
      let startAt = playheadRef.current;

      if (startAt < now + PLAYBACK_LEAD_S || startAt > now + PLAYBACK_MAX_LAG_S) {
        startAt = now + PLAYBACK_LEAD_S;
      }

      source.start(startAt);
      playheadRef.current = startAt + buffer.duration;
    };

    const handleStart = (payload: Uint8Array) => {
      // A previous transmission that never ended (device reset, dropped socket)
      // would otherwise leak its packets into this clip.
      finishReception();

      const meta = parseAudioMeta(payload);

      receptionRef.current = {
        id: String(meta?.id ?? 'car'),
        chunks: [],
        samples: 0,
        receivedAt: Date.now(),
        announcedDurationMs: 0,
      };

      playheadRef.current = 0;
      setRxActive(true);
      setRxLevel(0);
    };

    const handleChunk = (payload: Uint8Array) => {
      const samples = decodePcmFrame(payload);
      if (samples.length === 0) return;

      const reception =
        receptionRef.current ??
        ({
          id: `car-${Date.now()}`,
          chunks: [],
          samples: 0,
          receivedAt: Date.now(),
          announcedDurationMs: 0,
        } satisfies Reception);

      receptionRef.current = reception;
      reception.chunks.push(samples);
      reception.samples += samples.length;

      scheduleLivePlayback(samples);

      setRxActive(true);

      const now = performance.now();
      if (now - rxLevelAtRef.current > RX_LEVEL_INTERVAL_MS) {
        rxLevelAtRef.current = now;
        setRxLevel(audioLevel(samples));
      }

      // The device stops without an "end" message when the socket dies mid-clip,
      // so a silent gap also closes the reception.
      if (stallTimer !== null) clearTimeout(stallTimer);
      stallTimer = setTimeout(() => {
        stallTimer = null;
        if (!disposed) finishReception();
      }, RX_STALL_MS);
    };

    const handleEnd = (payload: Uint8Array) => {
      const meta = parseAudioMeta(payload);
      const reception = receptionRef.current;

      if (reception && typeof meta?.durationMs === 'number') {
        reception.announcedDurationMs = meta.durationMs;
      }

      finishReception();
    };

    const client = mqtt.connect(BROKER_URL, {
      clientId: `nextjs-walkie-${Math.random().toString(16).substring(2, 8)}`,
      clean: true,
    });

    client.on('connect', () => {
      if (disposed) return;

      isOfflineReported = false;
      clearOfflineTimer();
      setConnectionStatus('connected');

      client.subscribe([`${MIC_TOPIC_ROOT}/start`, `${MIC_TOPIC_ROOT}/chunk`, `${MIC_TOPIC_ROOT}/end`]);
    });

    client.on('message', (topic, payload) => {
      if (disposed) return;

      if (topic === `${MIC_TOPIC_ROOT}/start`) {
        handleStart(payload);
      } else if (topic === `${MIC_TOPIC_ROOT}/chunk`) {
        handleChunk(payload);
      } else if (topic === `${MIC_TOPIC_ROOT}/end`) {
        handleEnd(payload);
      }
    });

    client.on('reconnect', () => {
      if (disposed) return;

      setConnectionStatus('connecting');

      offlineTimer = setTimeout(reportOffline, OFFLINE_GRACE_MS);
    });

    client.on('error', (cause) => {
      if (disposed) return;

      console.error('MQTT error:', cause);

      offlineTimer = setTimeout(reportOffline, OFFLINE_GRACE_MS);
    });

    client.on('offline', () => {
      if (disposed) return;

      setConnectionStatus('connecting');

      offlineTimer = setTimeout(reportOffline, OFFLINE_GRACE_MS);
    });

    clientRef.current = client;

    // A dropped socket ends whatever was in flight: the car stops getting packets
    // and releases the speaker on its own timeout, so both panels go back to idle.
    client.on('close', () => {
      if (disposed) return;

      setConnectionStatus('connecting');

      offlineTimer = setTimeout(reportOffline, OFFLINE_GRACE_MS);

      stopTransmission();
      finishReception();
    });

    return () => {
      disposed = true;
      clearOfflineTimer();

      if (stallTimer !== null) {
        clearTimeout(stallTimer);
        stallTimer = null;
      }

      clientRef.current = null;
      client.end(true);

      capturingRef.current = false;

      if (txTimerRef.current !== null) {
        clearInterval(txTimerRef.current);
        txTimerRef.current = null;
      }

      releaseMicrophone();

      // Every scheduled packet was created from this context, so it goes away with
      // the component.
      void audioContextRef.current?.close().catch(() => undefined);
      audioContextRef.current = null;

      for (const clip of clipsRef.current) {
        URL.revokeObjectURL(clip.url);
      }

      clipsRef.current = [];
    };
  }, [releaseMicrophone, stopTransmission]);

  // Browsers keep an AudioContext suspended until the page is touched, and a
  // suspended context silently drops every packet. The first interaction unlocks
  // playback so the car is audible without a dedicated button press.
  useEffect(() => {
    const unlock = () => {
      void ensureAudioContext();
    };

    window.addEventListener('pointerdown', unlock, { capture: true, once: true });
    window.addEventListener('keydown', unlock, { capture: true, once: true });

    return () => {
      window.removeEventListener('pointerdown', unlock, { capture: true });
      window.removeEventListener('keydown', unlock, { capture: true });
    };
  }, [ensureAudioContext]);

  const handlePressStart = (event: ReactPointerEvent<HTMLButtonElement>) => {
    event.preventDefault();

    // Capturing the pointer means sliding off the button does not count as a
    // release, and letting go outside it still stops the transmission.
    event.currentTarget.setPointerCapture(event.pointerId);

    void startTransmission();
  };

  const handlePressEnd = (event: ReactPointerEvent<HTMLButtonElement>) => {
    if (event.currentTarget.hasPointerCapture(event.pointerId)) {
      event.currentTarget.releasePointerCapture(event.pointerId);
    }

    pressHeldRef.current = false;
    stopTransmission();
  };

  const handleKeyDown = (event: ReactKeyboardEvent<HTMLButtonElement>) => {
    if (event.repeat) return;
    if (event.key !== ' ' && event.key !== 'Enter') return;

    // Space and Enter would otherwise also fire a click, which toggles nothing
    // here but would restart the transmission.
    event.preventDefault();

    void startTransmission();
  };

  const handleKeyUp = (event: ReactKeyboardEvent<HTMLButtonElement>) => {
    if (event.key !== ' ' && event.key !== 'Enter') return;

    event.preventDefault();

    pressHeldRef.current = false;
    stopTransmission();
  };

  const isConnected = connectionStatus === 'connected';
  const isLive = txState === 'live';
  const isArming = txState === 'arming';
  const remainingMs = Math.max(0, WALKIE_MAX_RECORD_MS - txElapsedMs);
  const status = STATUS_BADGE[connectionStatus];

  return (
    <div className="space-y-4">
      <Card>
        <CardHeader>
          <CardTitle className="flex items-center gap-2">
            <Headphones className="size-4 text-muted-foreground" />
            Car Microphone
          </CardTitle>
          <CardDescription>
            Everything the INMP441 picks up, streamed live over MQTT.
          </CardDescription>
          <CardAction>
            <Badge variant="outline" className={cn('gap-1.5', status.className)}>
              {connectionStatus === 'connecting' ? (
                <Loader2 className="size-3 animate-spin" />
              ) : (
                <span className="size-1.5 rounded-full bg-current" />
              )}
              {status.label}
            </Badge>
          </CardAction>
        </CardHeader>

        <CardContent className="space-y-3">
          {audioBlocked ? (
            <Alert>
              <Volume2 />
              <AlertTitle>Audio is muted by the browser</AlertTitle>
              <AlertDescription>
                Browsers only start playing sound after a tap or a keypress.
                <AlertAction>
                  <Button
                    variant="outline"
                    size="xs"
                    onClick={() => {
                      void ensureAudioContext();
                    }}
                  >
                    Enable
                  </Button>
                </AlertAction>
              </AlertDescription>
            </Alert>
          ) : null}

          <div
            className={cn(
              'flex items-center gap-3 rounded-xl border p-3 transition-colors',
              rxActive ? 'border-emerald-500/30 bg-emerald-500/5' : 'bg-muted/40'
            )}
            aria-live="polite"
          >
            <span
              className={cn(
                'flex size-10 shrink-0 items-center justify-center rounded-lg transition-colors',
                rxActive ? 'bg-emerald-500 text-white' : 'bg-muted text-muted-foreground'
              )}
            >
              {rxActive ? <AudioLines className="size-5" /> : <Headphones className="size-5" />}
            </span>

            <div className="min-w-0 flex-1">
              <p className="text-sm font-medium">{rxActive ? 'Car is talking' : 'Channel idle'}</p>
              <p className="font-mono text-[11px] text-muted-foreground">
                {(WALKIE_SAMPLE_RATE / 1000).toFixed(0)} kHz · mono · 16 bit
              </p>
            </div>

            <LevelBar level={rxLevel} active={rxActive} className="w-20 shrink-0" />
          </div>

          <div className="space-y-2">
            <div className="flex items-center justify-between gap-2">
              <p className="text-xs font-medium text-muted-foreground">Recent transmissions</p>
              <span className="font-mono text-[11px] text-muted-foreground">
                {clips.length}/{MAX_CLIPS}
              </span>
            </div>

            {clips.length === 0 ? (
              <p className="rounded-lg border border-dashed px-3 py-5 text-center text-xs text-balance text-muted-foreground">
                Recordings from the car land here so you can replay or save them.
              </p>
            ) : (
              <ul className="space-y-2">
                {clips.map((clip) => (
                  <li key={clip.key} className="space-y-2 rounded-lg border bg-background p-2.5">
                    <div className="flex items-center gap-2">
                      <div className="min-w-0 flex-1">
                        <p className="text-sm font-medium">
                          {formatDuration(clip.durationMs)} transmission
                        </p>
                        <p className="font-mono text-[11px] text-muted-foreground">
                          {formatClock(clip.receivedAt)}
                        </p>
                      </div>

                      <Button asChild variant="ghost" size="icon-sm" className="shrink-0">
                        <a
                          href={clip.url}
                          download={`walkie-${clip.key}.wav`}
                          aria-label="Download transmission"
                        >
                          <Download />
                        </a>
                      </Button>
                    </div>

                    <audio controls preload="none" src={clip.url} className="h-9 w-full" />
                  </li>
                ))}
              </ul>
            )}
          </div>
        </CardContent>

        <CardFooter className="justify-between gap-3 text-xs text-muted-foreground">
          <span className="flex shrink-0 items-center gap-1.5">
            <Radio className="size-3.5" />
            MQTT topic
          </span>
          <span className="truncate font-mono text-[11px]">{MIC_TOPIC_ROOT}/chunk</span>
        </CardFooter>
      </Card>

      <Card>
        <CardHeader>
          <CardTitle className="flex items-center gap-2">
            <Mic className="size-4 text-muted-foreground" />
            Push to Talk
          </CardTitle>
          <CardDescription>
            Hold the button and your voice plays out of the car&apos;s speaker.
          </CardDescription>
          <CardAction>
            <Badge variant="outline" className="gap-1.5">
              <Radio className="size-3" />
              {formatDuration(WALKIE_MAX_RECORD_MS)} max
            </Badge>
          </CardAction>
        </CardHeader>

        <CardContent className="space-y-3">
          {error ? (
            <Alert variant="destructive">
              <WifiOff />
              <AlertTitle>Cannot transmit</AlertTitle>
              <AlertDescription>{error}</AlertDescription>
            </Alert>
          ) : null}

          <button
            type="button"
            onPointerDown={handlePressStart}
            onPointerUp={handlePressEnd}
            onPointerCancel={handlePressEnd}
            onKeyDown={handleKeyDown}
            onKeyUp={handleKeyUp}
            onContextMenu={(event) => event.preventDefault()}
            disabled={!isConnected}
            aria-pressed={isLive}
            aria-label="Hold to talk to the car"
            className={cn(
              // touch-none keeps the long press from turning into a page scroll,
              // select-none keeps it from turning into a text selection.
              'flex h-24 w-full touch-none flex-col items-center justify-center gap-1.5 rounded-2xl border text-sm font-medium transition-all outline-none select-none',
              'focus-visible:border-ring focus-visible:ring-3 focus-visible:ring-ring/50',
              isLive
                ? 'scale-[0.98] border-destructive/40 bg-destructive/10 text-destructive'
                : 'border-input bg-background shadow-xs hover:bg-muted active:scale-[0.98]',
              (isArming || !isConnected) && 'opacity-60'
            )}
          >
            {isArming ? (
              <Loader2 className="size-6 animate-spin" />
            ) : isLive ? (
              <CircleStop className="size-6" />
            ) : (
              <Mic className="size-6" />
            )}

            <span>
              {isLive ? 'Talking, release to send' : isArming ? 'Opening microphone' : 'Hold to talk'}
            </span>

            <span className="text-[11px] font-normal opacity-70">
              {isLive
                ? `${formatDuration(remainingMs)} left`
                : `up to ${formatDuration(WALKIE_MAX_RECORD_MS)}`}
            </span>
          </button>

          <div className="space-y-1.5">
            <div className="h-1.5 overflow-hidden rounded-full bg-muted">
              <div
                className={cn(
                  'h-full rounded-full transition-[width] duration-75',
                  isLive ? 'bg-destructive' : 'bg-primary'
                )}
                style={{ width: `${Math.min(100, (txElapsedMs / WALKIE_MAX_RECORD_MS) * 100)}%` }}
              />
            </div>

            <div className="flex items-center justify-between gap-2 font-mono text-[11px] text-muted-foreground">
              <span>
                {isLive ? 'transmitting' : txElapsedMs > 0 ? 'last take' : 'ready'}
              </span>
              <span>
                {isLive || txElapsedMs > 0 ? formatDuration(txElapsedMs) : '—'}
              </span>
            </div>
          </div>

          <div className="flex items-center gap-3">
            <span className="shrink-0 text-xs text-muted-foreground">Mic</span>
            <LevelBar level={txLevel} active={isLive} className="flex-1" />
            <span className="shrink-0 font-mono text-[11px] text-muted-foreground">
              {WALKIE_CHUNK_BYTES / 1024} kB packets
            </span>
          </div>
        </CardContent>

        <CardFooter className="justify-between gap-3 text-xs text-muted-foreground">
          <span className="flex shrink-0 items-center gap-1.5">
            <Radio className="size-3.5" />
            MQTT topic
          </span>
          <span className="truncate font-mono text-[11px]">{SPEAKER_TOPIC_ROOT}/chunk</span>
        </CardFooter>
      </Card>
    </div>
  );
}

/** Segment meter mirroring the firmware's serial level bar (-60..0 dBFS). */
function LevelBar({
  level,
  active,
  className,
}: {
  level: number;
  active: boolean;
  className?: string;
}) {
  const filled = Math.round(Math.min(1, Math.max(0, level)) * LEVEL_SEGMENTS);

  return (
    <div className={cn('flex items-end gap-0.5', className)} aria-hidden="true">
      {Array.from({ length: LEVEL_SEGMENTS }, (_, index) => {
        const isLit = active && index < filled;

        return (
          <span
            key={index}
            className={cn(
              'h-3 flex-1 rounded-[1px] transition-colors',
              !isLit
                ? 'bg-muted-foreground/20'
                : index >= LEVEL_SEGMENTS * 0.85
                  ? 'bg-destructive'
                  : index >= LEVEL_SEGMENTS * 0.6
                    ? 'bg-amber-500'
                    : 'bg-emerald-500'
            )}
          />
        );
      })}
    </div>
  );
}

/** "14:32:07" — only rendered on the client, after a clip has arrived. */
function formatClock(timestamp: number): string {
  const date = new Date(timestamp);

  return [date.getHours(), date.getMinutes(), date.getSeconds()]
    .map((part) => String(part).padStart(2, '0'))
    .join(':');
}

/** Optional fields published on the audio `start` / `end` topics. */
type AudioMeta = {
  id?: string;
  durationMs?: number;
  sampleRate?: number;
  channels?: number;
  bits?: number;
};

/** Decodes the small JSON blob the car sends on `.../audio/mic/start|end`. */
function parseAudioMeta(payload: Uint8Array): AudioMeta | null {
  if (payload.length === 0) return null;

  try {
    const parsed: unknown = JSON.parse(new TextDecoder().decode(payload));

    return typeof parsed === 'object' && parsed !== null ? (parsed as AudioMeta) : null;
  } catch {
    // A malformed payload must not kill the stream: the packets carry everything
    // the player needs anyway, so the clip is just timestamped as "car".
    return null;
  }
}

/**
 * Splits a `.../audio/mic/chunk` frame into samples: 2 bytes of little endian
 * sequence number followed by little endian 16 bit PCM (see flushMicChunk() in
 * the firmware).
 */
function decodePcmFrame(payload: Uint8Array): Int16Array {
  const dataBytes = payload.length - AUDIO_HEADER_BYTES;

  // A frame without the header, or with half a sample left, is not playable.
  if (dataBytes < 2) return new Int16Array(0);

  const view = new DataView(
    payload.buffer,
    payload.byteOffset + AUDIO_HEADER_BYTES,
    dataBytes - (dataBytes % 2)
  );

  const samples = new Int16Array(view.byteLength / 2);

  for (let i = 0; i < samples.length; i++) {
    samples[i] = view.getInt16(i * 2, true);
  }

  return samples;
}

