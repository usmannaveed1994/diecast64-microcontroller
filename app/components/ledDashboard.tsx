'use client';

import { useState, useEffect, useRef } from 'react';
import mqtt, { MqttClient } from 'mqtt';
import { Lightbulb, LightbulbOff, Loader2, Radio, WifiOff } from 'lucide-react';
import { Alert, AlertDescription, AlertTitle } from '@/components/ui/alert';
import { Badge } from '@/components/ui/badge';
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
const TOPIC_PREFIX = 'usman-esp32-9988/led';

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
      mqttClient.subscribe(`${TOPIC_PREFIX}/#`);
    });

    mqttClient.on('reconnect', () => {
      if (disposed || isOfflineReported) return;
      setConnectionStatus('connecting');
    });

    mqttClient.on('message', (topic, payload) => {
      if (disposed) return;
      const messageStr = payload.toString().trim().toUpperCase();
      if (topic === `${TOPIC_PREFIX}/a/state`) {
        setLeds((prev) => ({ ...prev, a: messageStr === 'ON' }));
      } else if (topic === `${TOPIC_PREFIX}/b/state`) {
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
    clientRef.current?.publish(`${TOPIC_PREFIX}/${target}/set`, isOn ? 'ON' : 'OFF');
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
          <span className="truncate font-mono text-[11px]">{TOPIC_PREFIX}</span>
        </CardFooter>
      </Card>

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