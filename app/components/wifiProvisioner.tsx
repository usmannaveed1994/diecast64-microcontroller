'use client';

import { useState } from 'react';
import { AlertTriangle, CheckCircle2, Eye, EyeOff, Loader2, Lock, Router, Send, Wifi } from 'lucide-react';
import { Alert, AlertDescription, AlertTitle } from '@/components/ui/alert';
import { Button } from '@/components/ui/button';
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from '@/components/ui/card';
import { Input } from '@/components/ui/input';
import { Label } from '@/components/ui/label';

type Status = {
  tone: 'success' | 'error';
  title: string;
  message: string;
};

export default function WifiProvisioner() {
  const [ssid, setSsid] = useState('');
  const [password, setPassword] = useState('');
  const [showPassword, setShowPassword] = useState(false);
  const [status, setStatus] = useState<Status | null>(null);
  const [loading, setLoading] = useState(false);

  const handleProvision = async (e: React.FormEvent) => {
    e.preventDefault();
    setLoading(true);
    setStatus(null);

    try {
      // Sends credentials over the air to the ESP32 captive IP
      const res = await fetch('http://192.168.4.1/api/wifi', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ ssid, password }),
      });

      if (res.ok) {
        setStatus({
          tone: 'success',
          title: 'Credentials received',
          message: 'ESP32 is rebooting and connecting to your router.',
        });
      } else {
        setStatus({
          tone: 'error',
          title: 'Could not save credentials',
          message: 'The ESP32 rejected the request. Please try again.',
        });
      }
    } catch (err) {
      console.error(err);
      setStatus({
        tone: 'error',
        title: 'Connection failed',
        message: 'Make sure you are connected to the "ESP32-Setup-AP" Wi-Fi network.',
      });
    } finally {
      setLoading(false);
    }
  };

  return (
    <Card>
      <CardHeader>
        <CardTitle className="flex items-center gap-2">
          <Router className="size-4 text-muted-foreground" />
          Wi-Fi Setup
        </CardTitle>
        <CardDescription>Save your router credentials so the ESP32 can go online.</CardDescription>
      </CardHeader>

      <CardContent className="space-y-4">
        <Alert>
          <Wifi />
          <AlertTitle>Join the setup network first</AlertTitle>
          <AlertDescription>
            Connect your phone to <span className="font-medium text-foreground">ESP32-Setup-AP</span>,
            then submit the form below.
          </AlertDescription>
        </Alert>

        <form onSubmit={handleProvision} className="space-y-4">
          <div className="space-y-2">
            <Label htmlFor="ssid">Network SSID</Label>
            <div className="relative">
              <Wifi className="pointer-events-none absolute top-1/2 left-3 size-4 -translate-y-1/2 text-muted-foreground" />
              <Input
                id="ssid"
                value={ssid}
                onChange={(e) => setSsid(e.target.value)}
                required
                placeholder="e.g. 186 GF 4G"
                autoComplete="off"
                autoCapitalize="none"
                spellCheck={false}
                className="h-11 pl-9 text-base md:text-base"
              />
            </div>
          </div>

          <div className="space-y-2">
            <Label htmlFor="password">Network Password</Label>
            <div className="relative">
              <Lock className="pointer-events-none absolute top-1/2 left-3 size-4 -translate-y-1/2 text-muted-foreground" />
              <Input
                id="password"
                type={showPassword ? 'text' : 'password'}
                value={password}
                onChange={(e) => setPassword(e.target.value)}
                required
                placeholder="••••••••"
                autoComplete="new-password"
                className="h-11 pr-10 pl-9 text-base md:text-base"
              />
              <Button
                type="button"
                variant="ghost"
                size="icon-sm"
                onClick={() => setShowPassword((visible) => !visible)}
                aria-label={showPassword ? 'Hide password' : 'Show password'}
                className="absolute top-1/2 right-1.5 -translate-y-1/2 text-muted-foreground"
              >
                {showPassword ? <EyeOff /> : <Eye />}
              </Button>
            </div>
          </div>

          <Button type="submit" size="lg" disabled={loading} className="h-11 w-full">
            {loading ? <Loader2 className="animate-spin" /> : <Send />}
            {loading ? 'Sending…' : 'Save & connect ESP32'}
          </Button>
        </form>

        {status && (
          <Alert
            variant={status.tone === 'error' ? 'destructive' : 'default'}
            className={
              status.tone === 'success'
                ? 'border-emerald-500/30 bg-emerald-500/5 text-emerald-700'
                : undefined
            }
          >
            {status.tone === 'success' ? <CheckCircle2 /> : <AlertTriangle />}
            <AlertTitle>{status.title}</AlertTitle>
            <AlertDescription
              className={status.tone === 'success' ? 'text-emerald-700/80' : undefined}
            >
              {status.message}
            </AlertDescription>
          </Alert>
        )}
      </CardContent>
    </Card>
  );
}
