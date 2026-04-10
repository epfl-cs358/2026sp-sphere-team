/** Developed with AI assistance (Claude, Anthropic) */

import { WebSocketServer, WebSocket } from "ws";

const PORT = Number(process.env.RELAY_PORT ?? 3001);

const wss = new WebSocketServer({ port: PORT });

const producers = new Set<WebSocket>();
const consumers = new Set<WebSocket>();

wss.on("connection", (ws, req) => {
  const url = new URL(req.url ?? "/", `http://localhost:${PORT}`);
  const role = url.searchParams.get("role");

  if (role === "producer") {
    producers.add(ws);
    console.log(`producer connected (${producers.size} total)`);

    ws.on("message", (data) => {
      for (const consumer of consumers) {
        if (consumer.readyState === WebSocket.OPEN) {
          consumer.send(data);
        }
      }
    });

    ws.on("close", () => {
      producers.delete(ws);
      console.log(`producer disconnected (${producers.size} total)`);
    });
  } else {
    consumers.add(ws);
    console.log(`consumer connected (${consumers.size} total)`);

    ws.on("close", () => {
      consumers.delete(ws);
      console.log(`consumer disconnected (${consumers.size} total)`);
    });
  }
});

console.log(`relay server listening on ws://localhost:${PORT}`);
console.log(`  producer: ws://localhost:${PORT}?role=producer`);
console.log(`  consumer: ws://localhost:${PORT}`);
