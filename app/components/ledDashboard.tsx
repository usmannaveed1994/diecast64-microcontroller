'use client';

import { useState, useEffect, useRef } from 'react';
import mqtt, { MqttClient } from 'mqtt';
import {
  Eraser,
  Lightbulb,
  LightbulbOff,
  Loader2,
  MonitorSmartphone,
  Radio,
  Send,
  Type,
  WifiOff,
} from 'lucide-react';
import { Alert, AlertDescription, AlertTitle } from '@/components/ui/alert';
import { Badge } from '@/components/ui/badge';
import { Button } from '@/components/ui/button';
import { Input } from '@/components/ui/input';
import {
  Card,
  CardAction,
  CardContent,
  CardDescription,
  CardFooter,
  CardHeader,
  CardTitle,
} from '@/components/ui/card';
import { Switch } from '@/components/ui/switch';
import { cn } from '@/lib/utils';

const BROKER_URL = 'wss://broker.hivemq.com:8884/mqtt';
const DEVICE_ROOT = 'usman-esp32-9988';
const LED_TOPIC_PREFIX = `${DEVICE_ROOT}/led`;
const TEXT_TOPIC_PREFIX = `${DEVICE_ROOT}/text`;

// TFT limits mirrored from firmware/microcontroller/race_car_main.ino: the firmware
// draws a 128x128 sprite (setRotation(5)) and puts the floating text in a 24 px band
// along the bottom. Font 2 glyphs are 8x16 px, so ~15 characters fit on one line and
// anything longer is marqueed across the panel.
const TFT_SPRITE_WIDTH = 128;
const TFT_TEXT_BAND_HEIGHT = 24;
const TFT_FONT_WIDTH = 8;
const FLOATING_TEXT_MAX_LENGTH = 64;

// mqtt.js reports `offline`/`error` for any dropped socket, including a single failed
// handshake. Wait a moment before surfacing the destructive alert so a quick reconnect
// (e.g. right after a page reload) does not flash "Broker unreachable".
const OFFLINE_GRACE_MS = 2000;

type ChannelId = 'a' | 'b';
type ConnectionStatus = 'connecting' | 'connected' | 'offline';

const CHANNELS: { id: ChannelId; label: string; pin: string }[] = [
  { id: 'a', label: 'LED A', pin: 'GPIO 25' },
  { id: 'b', label: 'LED B', pin: 'GPIO 26' },
];

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

export default function LedDashboard() {
  const clientRef = useRef<MqttClient | null>(null);
  const [connectionStatus, setConnectionStatus] = useState<ConnectionStatus>('connecting');
  const [leds, setLeds] = useState<Record<ChannelId, boolean>>({ a: false, b: false });
  const [textDraft, setTextDraft] = useState('');
  const [tftText, setTftText] = useState('');

  useEffect(() => {
    let disposed = false;
    let offlineTimer: ReturnType<typeof setTimeout> | null = null;
    let isOfflineReported = false;

    const clearOfflineTimer = () => {
      if (offlineTimer === null) return;
      clearTimeout(offlineTimer);
      offlineTimer = null;
    };

    const reportDisconnected = () => {
      if (disposed || isOfflineReported || offlineTimer !== null) return;
      setConnectionStatus('connecting');
      offlineTimer = setTimeout(() => {
        offlineTimer = null;
        isOfflineReported = true;
        if (!disposed) setConnectionStatus('offline');
      }, OFFLINE_GRACE_MS);
    };

    const mqttClient = mqtt.connect(BROKER_URL, {
      clientId: `nextjs-client-${Math.random().toString(16).substring(2, 8)}`,
      clean: true,
    });

    mqttClient.on('connect', () => {
      if (disposed) return;
      clearOfflineTimer();
      isOfflineReported = false;
      setConnectionStatus('connected');
      // Only the topics this panel renders: `#` would also deliver the walkie-talkie
      // audio (see walkieTalkie.tsx), whose binary packets get stringified ~30x a
      // second by the message handler below for nothing.
      mqttClient.subscribe([
        `${LED_TOPIC_PREFIX}/a/state`,
        `${LED_TOPIC_PREFIX}/b/state`,
        `${TEXT_TOPIC_PREFIX}/state`,
      ]);
    });

    mqttClient.on('reconnect', () => {
      if (disposed || isOfflineReported) return;
      setConnectionStatus('connecting');
    });

    mqttClient.on('message', (topic, payload) => {
      if (disposed) return;

      if (topic === `${TEXT_TOPIC_PREFIX}/state`) {
        // The device echoes the message it is rendering so the UI stays in sync,
        // including retained text restored after a reboot.
        setTftText(payload.toString());
        return;
      }

      const messageStr = payload.toString().trim().toUpperCase();
      if (topic === `${LED_TOPIC_PREFIX}/a/state`) {
        setLeds((prev) => ({ ...prev, a: messageStr === 'ON' }));
      } else if (topic === `${LED_TOPIC_PREFIX}/b/state`) {
        setLeds((prev) => ({ ...prev, b: messageStr === 'ON' }));
      }
    });

    mqttClient.on('offline', () => {
      reportDisconnected();
    });

    mqttClient.on('error', (err) => {
      console.error('MQTT error:', err);
      reportDisconnected();
    });

    clientRef.current = mqttClient;

    return () => {
      // A disposed client must never update state: in development React Strict Mode
      // remounts the effect, and the ended first client still emits `error`
      // ("client disconnecting") when its in-flight subscribe is processed. Without
      // this guard that stale event briefly rendered the Offline badge and alert.
      disposed = true;
      clearOfflineTimer();
      clientRef.current = null;
      mqttClient.end(true);
    };
  }, []);

  const setLed = (target: ChannelId, isOn: boolean) => {
    // Optimistic update – the device's "<topic>/state" message stays the source of truth.
    setLeds((prev) => ({ ...prev, [target]: isOn }));
    clientRef.current?.publish(`${LED_TOPIC_PREFIX}/${target}/set`, isOn ? 'ON' : 'OFF');
  };

  const publishFloatingText = (text: string) => {
    const client = clientRef.current;
    if (!client || connectionStatus !== 'connected') return;

    // Retained so the TFT restores the message after a reboot or a broker reconnect.
    client.publish(`${TEXT_TOPIC_PREFIX}/set`, text, { qos: 0, retain: true });
    // Optimistic update – the device echoes "<prefix>/text/state" once it renders it.
    setTftText(text);
  };

  const sendFloatingText = () => publishFloatingText(textDraft.trim());

  const clearFloatingText = () => {
    setTextDraft('');
    publishFloatingText('');
  };

  const isDisconnected = connectionStatus !== 'connected';
  const statusBadge = STATUS_BADGE[connectionStatus];

  return (
    <div className="space-y-4">
      <Card>
        <CardHeader>
          <CardTitle className="flex items-center gap-2">
            <Lightbulb className="size-4 text-muted-foreground" />
            LED Control
          </CardTitle>
          <CardDescription>Toggle each channel over MQTT.</CardDescription>
          <CardAction>
            <Badge variant="outline" className={cn('gap-1.5', statusBadge.className)}>
              {connectionStatus === 'connecting' ? (
                <Loader2 className="size-3 animate-spin" />
              ) : (
                <span className="size-1.5 rounded-full bg-current" />
              )}
              {statusBadge.label}
            </Badge>
          </CardAction>
        </CardHeader>

        <CardContent className="space-y-2">
          {CHANNELS.map(({ id, label, pin }) => {
            const isOn = leds[id];

            return (
              <label
                key={id}
                htmlFor={`led-${id}`}
                className={cn(
                  'flex cursor-pointer items-center gap-3 rounded-xl border p-3 transition-colors',
                  isOn ? 'border-primary/30 bg-primary/5' : 'bg-background hover:bg-muted/60',
                  isDisconnected && 'cursor-not-allowed opacity-60'
                )}
              >
                <span
                  className={cn(
                    'flex size-10 shrink-0 items-center justify-center rounded-lg transition-colors',
                    isOn ? 'bg-primary text-primary-foreground' : 'bg-muted text-muted-foreground'
                  )}
                >
                  {isOn ? <Lightbulb className="size-5" /> : <LightbulbOff className="size-5" />}
                </span>

                <span className="min-w-0 flex-1">
                  <span className="flex items-baseline gap-1.5">
                    <span className="text-sm font-medium">{label}</span>
                    <span className="font-mono text-[11px] text-muted-foreground">{pin}</span>
                  </span>
                  <span
                    className={cn(
                      'block text-xs',
                      isOn ? 'font-medium text-primary' : 'text-muted-foreground'
                    )}
                  >
                    {isOn ? 'Active · ON' : 'Inactive · OFF'}
                  </span>
                </span>

                <Switch
                  id={`led-${id}`}
                  checked={isOn}
                  onCheckedChange={(checked) => setLed(id, checked)}
                  disabled={isDisconnected}
                  aria-label={`Toggle ${label}`}
                />
              </label>
            );
          })}
        </CardContent>

        <CardFooter className="justify-between gap-3 text-xs text-muted-foreground">
          <span className="flex shrink-0 items-center gap-1.5">
            <Radio className="size-3.5" />
            MQTT topic
          </span>
          <span className="truncate font-mono text-[11px]">{LED_TOPIC_PREFIX}</span>
        </CardFooter>
      </Card>

      <FloatingTextCard
        value={textDraft}
        displayedText={tftText}
        disabled={isDisconnected}
        onChange={setTextDraft}
        onSend={sendFloatingText}
        onClear={clearFloatingText}
      />

      {connectionStatus === 'offline' && (
        <Alert variant="destructive">
          <WifiOff />
          <AlertTitle>Broker unreachable</AlertTitle>
          <AlertDescription>
            Check your internet connection. Reconnecting to HiveMQ automatically…
          </AlertDescription>
        </Alert>
      )}
    </div>
  );
}

type FloatingTextCardProps = {
  value: string;
  displayedText: string;
  disabled: boolean;
  onChange: (value: string) => void;
  onSend: () => void;
  onClear: () => void;
};

/**
 * Publishes free text to the ESP32, which renders it over the 3D car on its
 * 128x128 TFT. Short messages are centred; longer ones marquee across the panel.
 */
function FloatingTextCard({
  value,
  displayedText,
  disabled,
  onChange,
  onSend,
  onClear,
}: FloatingTextCardProps) {
  const trimmed = value.trim();
  const canSend = !disabled && trimmed.length > 0;
  const isScrolling = trimmed.length * TFT_FONT_WIDTH > TFT_SPRITE_WIDTH;
  const hasTextOnTft = displayedText.length > 0;

  const handleSubmit = (event: React.FormEvent<HTMLFormElement>) => {
    event.preventDefault();
    if (canSend) onSend();
  };

  return (
    <Card>
      <CardHeader>
        <CardTitle className="flex items-center gap-2">
          <Type className="size-4 text-muted-foreground" />
          Floating Text
        </CardTitle>
        <CardDescription>Send a message to the TFT overlay.</CardDescription>
        <CardAction>
          <Badge variant="outline" className="gap-1.5">
            <MonitorSmartphone className="size-3" />
            128 × 128
          </Badge>
        </CardAction>
      </CardHeader>

      <CardContent className="space-y-3">
        <div className="flex gap-3">
          <TftPreview text={trimmed || displayedText} />

          <form onSubmit={handleSubmit} className="flex min-w-0 flex-1 flex-col gap-2">
            <Input
              id="floating-text"
              value={value}
              onChange={(event) => onChange(event.target.value)}
              maxLength={FLOATING_TEXT_MAX_LENGTH}
              placeholder="e.g. DIECAST 64"
              autoComplete="off"
              disabled={disabled}
              aria-label="Floating text message"
              className="h-10 text-base md:text-base"
            />

            <div className="flex gap-2">
              <Button type="submit" disabled={!canSend} className="h-10 flex-1">
                <Send />
                Send to TFT
              </Button>
              <Button
                type="button"
                variant="outline"
                size="icon"
                className="size-10"
                onClick={onClear}
                disabled={disabled || (!hasTextOnTft && trimmed.length === 0)}
                aria-label="Clear TFT text"
              >
                <Eraser />
              </Button>
            </div>

            <p className="text-[11px] text-muted-foreground">
              {value.length}/{FLOATING_TEXT_MAX_LENGTH} chars ·{' '}
              {isScrolling ? 'scrolls across the panel' : 'centred on the panel'}
            </p>
          </form>
        </div>

        <div className="flex items-center justify-between gap-3 rounded-lg border bg-muted/40 px-3 py-2 text-xs">
          <span className="shrink-0 text-muted-foreground">On TFT now</span>
          <span className="min-w-0 truncate font-mono text-[11px]">{displayedText || '—'}</span>
        </div>
      </CardContent>

      <CardFooter className="justify-between gap-3 text-xs text-muted-foreground">
        <span className="flex shrink-0 items-center gap-1.5">
          <Radio className="size-3.5" />
          MQTT topic
        </span>
        <span className="truncate font-mono text-[11px]">{TEXT_TOPIC_PREFIX}/set</span>
      </CardFooter>
    </Card>
  );
}

/** Scaled mock-up of the 128x128 TFT (full size from `sm` up), including the text band. */
function TftPreview({ text }: { text: string }) {
  return (
    <div
      aria-hidden="true"
      className="relative size-24 shrink-0 overflow-hidden rounded-lg border border-border bg-black sm:size-32"
    >
      <span className="absolute top-1.5 left-2 font-mono text-[7px] tracking-tight text-cyan-400/60">
        DIECAST64 GARAGE
      </span>
      <span className="absolute top-4 left-2 font-mono text-[6px] text-fuchsia-400/40">
        128 × 128 · ROT 5
      </span>

      <div
        className="absolute inset-x-0 bottom-0 flex items-center border-t border-fuchsia-500/60 px-1.5"
        style={{ height: TFT_TEXT_BAND_HEIGHT }}
      >
        {text ? (
          <span className="w-full truncate text-center font-mono text-[10px] text-cyan-300">
            {text}
          </span>
        ) : (
          <span className="w-full text-center font-mono text-[9px] text-muted-foreground/50">
            floating text
          </span>
        )}
      </div>
    </div>
  );
}
