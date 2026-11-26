'use client';

import { useState } from 'react';
import { Cpu, LayoutDashboard, Wifi } from 'lucide-react';
import LedDashboard from './components/ledDashboard';
import WalkieTalkie from './components/walkieTalkie';
import WifiProvisioner from './components/wifiProvisioner';
import { Tabs, TabsContent, TabsList, TabsTrigger } from '@/components/ui/tabs';

type TabValue = 'dashboard' | 'walkie' | 'provision';

export default function Home() {
  const [activeTab, setActiveTab] = useState<TabValue>('dashboard');

  return (
    <div className="flex min-h-dvh flex-col bg-secondary/70">
    <header className="sticky top-0 z-30 border-b-2 border-border bg-secondary backdrop-blur-xl">
      <div className="mx-auto flex w-full max-w-md items-center gap-3 px-4 py-3">
        <div className="flex size-9 shrink-0 items-center justify-center rounded-xl bg-white text-primary-foreground">
          <Cpu className="size-4.5" />
        </div>
        <div className="min-w-0 flex-1">
          <h1 className="truncate text-sm font-semibold tracking-tight">Diecast64 Control Center</h1>
          <p className="truncate text-xs text-muted-foreground">Live device control</p>
        </div>
        <a
          href="https://www.instagram.com/diecast64.pk"
          target="_blank"
          rel="noopener noreferrer"
          aria-label="Follow Diecast64 on Instagram"
          className="flex size-9 shrink-0 items-center justify-center rounded-md border-2 border-border bg-primary text-primary-foreground shadow-brutal-sm transition-transform hover:-translate-x-0.5 hover:-translate-y-0.5 hover:shadow-brutal active:translate-x-0.5 active:translate-y-0.5 active:shadow-none"
        >
          <svg
            className="size-4.5"
            viewBox="0 0 24 24"
            fill="none"
            stroke="currentColor"
            strokeWidth="2.2"
            strokeLinecap="round"
            strokeLinejoin="round"
            aria-hidden="true"
          >
            <rect width="20" height="20" x="2" y="2" rx="5" ry="5" />
            <path d="M16 11.37A4 4 0 1 1 12.63 8 4 4 0 0 1 16 11.37z" />
            <line x1="17.5" x2="17.51" y1="6.5" y2="6.5" />
          </svg>
        </a>
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

          <TabsList className="fixed inset-x-0 bottom-0 z-30 mx-auto h-auto! w-full max-w-md gap-1 rounded-none border-t border-border/60 bg-secondary p-2 pb-[max(0.5rem,env(safe-area-inset-bottom))] backdrop-blur-xl">
            <TabsTrigger
              value="dashboard"
              className="h-12 flex-1 flex-col gap-1 rounded-xl px-3 text-[11px] font-medium data-active:bg-primary data-active:text-primary-foreground"
            >
              <LayoutDashboard className="size-5" />
              Dashboard
            </TabsTrigger>
            <TabsTrigger
              value="provision"
              className="h-12 flex-1 flex-col gap-1 rounded-xl px-3 text-[11px] font-medium data-active:bg-primary data-active:text-primary-foreground"
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