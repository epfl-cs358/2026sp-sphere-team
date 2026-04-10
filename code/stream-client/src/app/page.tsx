/** Developed with AI assistance (Claude, Anthropic) */

"use client";

import { useRef, useState, useCallback, useEffect } from "react";

type Status = "idle" | "previewing" | "streaming";

export default function StreamClient() {
  const videoRef = useRef<HTMLVideoElement>(null);
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const wsRef = useRef<WebSocket | null>(null);
  const intervalRef = useRef<ReturnType<typeof setInterval> | null>(null);

  const [status, setStatus] = useState<Status>("idle");
  const [relayUrl, setRelayUrl] = useState("ws://localhost:3001");
  const [fps, setFps] = useState(15);
  const [quality, setQuality] = useState(0.7);
  const [resolution, setResolution] = useState<"480p" | "720p">("480p");
  const [frameCount, setFrameCount] = useState(0);

  const resolutions = {
    "480p": { width: 640, height: 480 },
    "720p": { width: 1280, height: 720 },
  };

  const startCamera = useCallback(async () => {
    const { width, height } = resolutions[resolution];
    const stream = await navigator.mediaDevices.getUserMedia({
      video: { width: { ideal: width }, height: { ideal: height } },
      audio: false,
    });

    if (videoRef.current) {
      videoRef.current.srcObject = stream;
      await videoRef.current.play();
    }

    setStatus("previewing");
  }, [resolution]);

  const stopCamera = useCallback(() => {
    if (videoRef.current?.srcObject) {
      const tracks = (videoRef.current.srcObject as MediaStream).getTracks();
      tracks.forEach((t) => t.stop());
      videoRef.current.srcObject = null;
    }
  }, []);

  const stopStreaming = useCallback(() => {
    if (intervalRef.current) {
      clearInterval(intervalRef.current);
      intervalRef.current = null;
    }
    if (wsRef.current) {
      wsRef.current.close();
      wsRef.current = null;
    }
    setStatus("previewing");
    setFrameCount(0);
  }, []);

  const startStreaming = useCallback(() => {
    const video = videoRef.current;
    const canvas = canvasRef.current;
    if (!video || !canvas) return;

    const ws = new WebSocket(`${relayUrl}?role=producer`);
    ws.binaryType = "arraybuffer";
    wsRef.current = ws;

    ws.onopen = () => {
      setStatus("streaming");

      const ctx = canvas.getContext("2d")!;

      intervalRef.current = setInterval(() => {
        canvas.width = video.videoWidth;
        canvas.height = video.videoHeight;
        ctx.drawImage(video, 0, 0);

        canvas.toBlob(
          (blob) => {
            if (blob && ws.readyState === WebSocket.OPEN) {
              blob.arrayBuffer().then((buf) => ws.send(buf));
              setFrameCount((c) => c + 1);
            }
          },
          "image/jpeg",
          quality
        );
      }, 1000 / fps);
    };

    ws.onclose = () => stopStreaming();
    ws.onerror = () => stopStreaming();
  }, [relayUrl, fps, quality, stopStreaming]);

  useEffect(() => {
    return () => {
      stopStreaming();
      stopCamera();
    };
  }, [stopStreaming, stopCamera]);

  return (
    <div className="flex flex-1 flex-col gap-6 p-6">
      <div className="flex items-center justify-between">
        <h1 className="text-xl font-semibold">Stream Client</h1>
        <div className="flex items-center gap-2">
          <span
            className={`inline-block h-2 w-2 rounded-full ${
              status === "streaming"
                ? "bg-red-500 animate-pulse"
                : status === "previewing"
                  ? "bg-green-500"
                  : "bg-zinc-600"
            }`}
          />
          <span className="text-sm text-zinc-400">
            {status === "streaming"
              ? `Streaming (${frameCount} frames)`
              : status === "previewing"
                ? "Camera ready"
                : "Idle"}
          </span>
        </div>
      </div>

      <div className="flex flex-1 gap-6">
        <div className="flex flex-1 items-center justify-center rounded-lg border border-zinc-800 bg-zinc-900 overflow-hidden">
          <video
            ref={videoRef}
            muted
            playsInline
            className="max-w-full max-h-full"
          />
          <canvas ref={canvasRef} className="hidden" />
        </div>

        <div className="flex w-64 flex-col gap-4">
          <div className="flex flex-col gap-2 rounded-lg border border-zinc-800 p-4">
            <label className="text-xs text-zinc-500">Relay URL</label>
            <input
              type="text"
              value={relayUrl}
              onChange={(e) => setRelayUrl(e.target.value)}
              disabled={status === "streaming"}
              className="rounded-md border border-zinc-700 bg-zinc-800 px-3 py-1.5 text-sm disabled:opacity-50"
            />
          </div>

          <div className="flex flex-col gap-2 rounded-lg border border-zinc-800 p-4">
            <label className="text-xs text-zinc-500">Resolution</label>
            <div className="flex gap-2">
              {(["480p", "720p"] as const).map((res) => (
                <button
                  key={res}
                  onClick={() => setResolution(res)}
                  disabled={status !== "idle"}
                  className={`flex-1 rounded px-2 py-1 text-sm transition-colors disabled:opacity-50 ${
                    resolution === res
                      ? "bg-zinc-700 text-zinc-100"
                      : "text-zinc-500 hover:text-zinc-300"
                  }`}
                >
                  {res}
                </button>
              ))}
            </div>
          </div>

          <div className="flex flex-col gap-2 rounded-lg border border-zinc-800 p-4">
            <label className="text-xs text-zinc-500">FPS: {fps}</label>
            <input
              type="range"
              min={5}
              max={30}
              value={fps}
              onChange={(e) => setFps(Number(e.target.value))}
              disabled={status === "streaming"}
              className="disabled:opacity-50"
            />
          </div>

          <div className="flex flex-col gap-2 rounded-lg border border-zinc-800 p-4">
            <label className="text-xs text-zinc-500">
              JPEG Quality: {Math.round(quality * 100)}%
            </label>
            <input
              type="range"
              min={0.1}
              max={1}
              step={0.1}
              value={quality}
              onChange={(e) => setQuality(Number(e.target.value))}
              disabled={status === "streaming"}
              className="disabled:opacity-50"
            />
          </div>

          <div className="flex flex-col gap-2 mt-auto">
            {status === "idle" && (
              <button
                onClick={startCamera}
                className="rounded-md bg-zinc-100 px-4 py-2 text-sm font-medium text-zinc-900 hover:bg-zinc-200 transition-colors"
              >
                Start Camera
              </button>
            )}

            {status === "previewing" && (
              <>
                <button
                  onClick={startStreaming}
                  className="rounded-md bg-red-600 px-4 py-2 text-sm font-medium text-white hover:bg-red-700 transition-colors"
                >
                  Start Streaming
                </button>
                <button
                  onClick={() => {
                    stopCamera();
                    setStatus("idle");
                  }}
                  className="rounded-md border border-zinc-700 px-4 py-2 text-sm text-zinc-400 hover:text-zinc-200 transition-colors"
                >
                  Stop Camera
                </button>
              </>
            )}

            {status === "streaming" && (
              <button
                onClick={stopStreaming}
                className="rounded-md border border-red-800 bg-red-950 px-4 py-2 text-sm font-medium text-red-400 hover:bg-red-900 transition-colors"
              >
                Stop Streaming
              </button>
            )}
          </div>
        </div>
      </div>
    </div>
  );
}
