'use client';

import { useState } from 'react';
import { AudioLines, Cpu, LayoutDashboard, Wifi } from 'lucide-react';
import LedDashboard from './components/ledDashboard';
import WalkieTalkie from './components/walkieTalkie';
import WifiProvisioner from './components/wifiProvisioner';
import { Tabs, TabsContent, TabsList, TabsTrigger } from '@/components/ui/tabs';

type TabValue = 'dashboard' | 'walkie' | 'provision';

export default function Home() {
  const [activeTab, setActiveTab] = useState<TabValue>('dashboard');

  return (
    <div className="flex min-h-dvh flex-col bg-muted/40">
      <header className="sticky top-0 z-30 border-b border-border/60 bg-background/80 backdrop-blur-xl">
        <div className="mx-auto flex w-full max-w-md items-center gap-3 px-4 py-3">
          <div className="flex size-9 shrink-0 items-center justify-center rounded-xl bg-primary text-primary-foreground">
            <Cpu className="size-4.5" />
          </div>
          <div className="min-w-0 flex-1">
            <h1 className="truncate text-sm font-semibold tracking-tight">ESP32 Control Center</h1>
            <p className="truncate text-xs text-muted-foreground">HiveMQ MQTT · Live device control</p>
          </div>
        </div>
      </header>

      <main className="mx-auto flex w-full max-w-md flex-1 flex-col px-4 pt-4 pb-28">
        <Tabs
          value={activeTab}
          onValueChange={(value) => setActiveTab(value as TabValue)}
          className="flex-1 gap-4"
        >
          <TabsContent
            value="dashboard"
            forceMount
            className="flex-1 data-[state=inactive]:hidden"
          >
            <LedDashboard />
          </TabsContent>
          <TabsContent
            value="walkie"
            forceMount
            className="flex-1 data-[state=inactive]:hidden"
          >
            <WalkieTalkie />
          </TabsContent>
          <TabsContent
            value="provision"
            forceMount
            className="flex-1 data-[state=inactive]:hidden"
          >
            <WifiProvisioner />
          </TabsContent>

          <TabsList className="fixed inset-x-0 bottom-0 z-30 mx-auto h-auto! w-full max-w-md gap-1 rounded-none border-t border-border/60 bg-background/95 p-2 pb-[max(0.5rem,env(safe-area-inset-bottom))] backdrop-blur-xl">
            <TabsTrigger
              value="dashboard"
              className="h-12 flex-1 flex-col gap-1 rounded-xl px-3 text-[11px] font-medium data-active:bg-muted data-active:text-primary"
            >
              <LayoutDashboard className="size-5" />
              Dashboard
            </TabsTrigger>
            <TabsTrigger
              value="walkie"
              className="h-12 flex-1 flex-col gap-1 rounded-xl px-3 text-[11px] font-medium data-active:bg-muted data-active:text-primary"
            >
              <AudioLines className="size-5" />
              Walkie
            </TabsTrigger>
            <TabsTrigger
              value="provision"
              className="h-12 flex-1 flex-col gap-1 rounded-xl px-3 text-[11px] font-medium data-active:bg-muted data-active:text-primary"
            >
              <Wifi className="size-5" />
              Wi-Fi Setup
            </TabsTrigger>
          </TabsList>
        </Tabs>
      </main>
    </div>
  );
}