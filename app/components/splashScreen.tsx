'use client';

import Image from 'next/image';
import { useEffect, useState } from 'react';

const SPLASH_VISIBLE_MS = 2000;
const SPLASH_FADE_MS = 300;

type SplashPhase = 'visible' | 'fading' | 'hidden';

/**
 * Full-screen white splash shown on every hard page load. It is part of the
 * server-rendered HTML, so it covers the app from the very first paint; the
 * client then hides it after ~2s and unmounts it once the fade finishes.
 *
 * Timing is driven by inline styles/JS instead of a custom CSS animation so it
 * always runs, regardless of how/when the stylesheet is compiled or cached.
 */
export default function SplashScreen() {
  const [phase, setPhase] = useState<SplashPhase>('visible');

  useEffect(() => {
    const fadeTimer = setTimeout(() => setPhase('fading'), SPLASH_VISIBLE_MS);
    const unmountTimer = setTimeout(
      () => setPhase('hidden'),
      SPLASH_VISIBLE_MS + SPLASH_FADE_MS,
    );

    return () => {
      clearTimeout(fadeTimer);
      clearTimeout(unmountTimer);
    };
  }, []);

  if (phase === 'hidden') {
    return null;
  }

  const isVisible = phase === 'visible';

  return (
    <div
      aria-hidden="true"
      className="fixed inset-0 z-100 flex items-center justify-center bg-white"
      style={{
        opacity: isVisible ? 1 : 0,
        pointerEvents: isVisible ? 'auto' : 'none',
        transition: `opacity ${SPLASH_FADE_MS}ms ease-out`,
      }}
    >
      <Image
        src="/logo.svg"
        alt=""
        width={64}
        height={64}
        loading="eager"
        unoptimized
        className="size-16 drop-shadow-md"
      />
    </div>
  );
}
