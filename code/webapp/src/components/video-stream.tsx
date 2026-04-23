/** Developed with AI assistance (Claude, Anthropic) */

"use client";

import { useRef, useEffect, useCallback } from "react";
import { VideoSource } from "@/lib/video";

interface VideoStreamProps {
  source: VideoSource;
  url: string;
  className?: string;
}

export function VideoStream({ source, url, className }: VideoStreamProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null);

  const drawFrame = useCallback((img: HTMLImageElement) => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    canvas.width = img.naturalWidth || img.width;
    canvas.height = img.naturalHeight || img.height;

    const ctx = canvas.getContext("2d");
    ctx?.drawImage(img, 0, 0);
  }, []);

  useEffect(() => {
    source.connect(url, {
      onFrame: drawFrame,
      onError: (err) => console.error(`[${source.type}]`, err),
    });

    return () => {
      source.disconnect();
    };
  }, [source, url, drawFrame]);

  return (
    <canvas
      ref={canvasRef}
      className={className}
    />
  );
}
