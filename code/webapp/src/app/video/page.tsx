/** Developed with AI assistance (Claude, Anthropic) */

"use client";

import { useState, useMemo } from "react";
import { VideoStream } from "@/components/video-stream";
import {
  MockSource,
  MJPEGSource,
  WebSocketSource,
  type VideoSource,
} from "@/lib/video";

type SourceType = "mock" | "mjpeg" | "websocket";

const SOURCE_LABELS: Record<SourceType, string> = {
  mock: "Mock",
  mjpeg: "MJPEG (HTTP)",
  websocket: "WebSocket",
};

export default function VideoPage() {
  const [sourceType, setSourceType] = useState<SourceType>("mock");
  const [url, setUrl] = useState("");
  const [activeConnection, setActiveConnection] = useState<{
    type: SourceType;
    url: string;
  } | null>({ type: "mock", url: "" });

  const source: VideoSource | null = useMemo(() => {
    if (!activeConnection) return null;
    switch (activeConnection.type) {
      case "mjpeg":
        return new MJPEGSource();
      case "websocket":
        return new WebSocketSource();
      default:
        return new MockSource();
    }
  }, [activeConnection]);

  const connect = () => {
    if (sourceType === "mock") {
      setActiveConnection({ type: "mock", url: "" });
    } else if (url.trim()) {
      setActiveConnection({ type: sourceType, url: url.trim() });
    }
  };

  const disconnect = () => {
    setActiveConnection(null);
  };

  const isConnected = activeConnection !== null;
  const needsUrl = sourceType !== "mock";

  return (
    <div className="flex flex-1 flex-col gap-6 p-6">
      <div className="flex items-center justify-between">
        <h1 className="text-xl font-semibold">Video Feed</h1>
        <div className="flex items-center gap-3">
          <div className="flex rounded-md border border-zinc-800 text-sm">
            {(Object.keys(SOURCE_LABELS) as SourceType[]).map((type) => (
              <button
                key={type}
                onClick={() => {
                  setSourceType(type);
                  if (isConnected) disconnect();
                }}
                className={`px-3 py-1.5 transition-colors ${
                  sourceType === type
                    ? "bg-zinc-800 text-zinc-100"
                    : "text-zinc-500 hover:text-zinc-300"
                }`}
              >
                {SOURCE_LABELS[type]}
              </button>
            ))}
          </div>

          {needsUrl && (
            <input
              type="text"
              value={url}
              onChange={(e) => setUrl(e.target.value)}
              onKeyDown={(e) => {
                if (e.key === "Enter") connect();
              }}
              placeholder={
                sourceType === "mjpeg"
                  ? "http://192.168.1.1/stream"
                  : "ws://localhost:3001"
              }
              className="rounded-md border border-zinc-800 bg-zinc-900 px-3 py-1.5 text-sm text-zinc-100 placeholder:text-zinc-600 w-72"
            />
          )}

          {isConnected ? (
            <button
              onClick={disconnect}
              className="rounded-md border border-red-800 bg-red-950 px-4 py-1.5 text-sm font-medium text-red-400 hover:bg-red-900 transition-colors"
            >
              Disconnect
            </button>
          ) : (
            <button
              onClick={connect}
              disabled={needsUrl && !url.trim()}
              className="rounded-md bg-zinc-100 px-4 py-1.5 text-sm font-medium text-zinc-900 hover:bg-zinc-200 transition-colors disabled:opacity-40 disabled:cursor-not-allowed"
            >
              Connect
            </button>
          )}
        </div>
      </div>

      <div className="flex flex-1 items-center justify-center rounded-lg border border-zinc-800 bg-zinc-900 overflow-hidden">
        {source && activeConnection ? (
          <VideoStream
            source={source}
            url={activeConnection.url}
            className="max-w-full max-h-full"
          />
        ) : (
          <p className="text-sm text-zinc-600">
            Select a source and connect
          </p>
        )}
      </div>
    </div>
  );
}
