import { describe, it, expect, beforeAll, afterAll } from "vitest";
import { WebSocketServer, WebSocket } from "ws";

let wss: WebSocketServer;
const PORT = 4567;

const producers = new Set<WebSocket>();
const consumers = new Set<WebSocket>();

function setupRelay(server: WebSocketServer) {
  server.on("connection", (ws, req) => {
    const url = new URL(req.url ?? "/", `http://localhost:${PORT}`);
    const role = url.searchParams.get("role");

    if (role === "producer") {
      producers.add(ws);
      ws.on("message", (data) => {
        for (const consumer of consumers) {
          if (consumer.readyState === WebSocket.OPEN) {
            consumer.send(data);
          }
        }
      });
      ws.on("close", () => producers.delete(ws));
    } else {
      consumers.add(ws);
      ws.on("close", () => consumers.delete(ws));
    }
  });
}

function connectWs(role?: string): Promise<WebSocket> {
  const query = role ? `?role=${role}` : "";
  return new Promise((resolve, reject) => {
    const ws = new WebSocket(`ws://localhost:${PORT}${query}`);
    ws.on("open", () => resolve(ws));
    ws.on("error", reject);
  });
}

describe("Relay Server", () => {
  beforeAll(() => {
    wss = new WebSocketServer({ port: PORT });
    setupRelay(wss);
  });

  afterAll(() => {
    for (const client of wss.clients) client.close();
    wss.close();
  });

  it("accepts producer connections", async () => {
    const ws = await connectWs("producer");
    expect(ws.readyState).toBe(WebSocket.OPEN);
    ws.close();
  });

  it("accepts consumer connections", async () => {
    const ws = await connectWs();
    expect(ws.readyState).toBe(WebSocket.OPEN);
    ws.close();
  });

  it("relays frames from producer to consumer", async () => {
    const producer = await connectWs("producer");
    const consumer = await connectWs();

    const received = new Promise<Buffer>((resolve) => {
      consumer.on("message", (data) => resolve(data as Buffer));
    });

    const testData = Buffer.from([0xff, 0xd8, 0xff, 0xe0]); // JPEG header
    producer.send(testData);

    const result = await received;
    expect(Buffer.from(result)).toEqual(testData);

    producer.close();
    consumer.close();
  });

  it("broadcasts to multiple consumers", async () => {
    const producer = await connectWs("producer");
    const consumer1 = await connectWs();
    const consumer2 = await connectWs();

    const received1 = new Promise<Buffer>((resolve) => {
      consumer1.on("message", (data) => resolve(data as Buffer));
    });
    const received2 = new Promise<Buffer>((resolve) => {
      consumer2.on("message", (data) => resolve(data as Buffer));
    });

    const testData = Buffer.from("test-frame");
    producer.send(testData);

    const [r1, r2] = await Promise.all([received1, received2]);
    expect(Buffer.from(r1)).toEqual(testData);
    expect(Buffer.from(r2)).toEqual(testData);

    producer.close();
    consumer1.close();
    consumer2.close();
  });

  it("does not relay to producers", async () => {
    const producer1 = await connectWs("producer");
    const producer2 = await connectWs("producer");

    let receivedByProducer2 = false;
    producer2.on("message", () => {
      receivedByProducer2 = true;
    });

    producer1.send(Buffer.from("test"));

    await new Promise((r) => setTimeout(r, 100));
    expect(receivedByProducer2).toBe(false);

    producer1.close();
    producer2.close();
  });

  it("cleans up producer set on disconnect", async () => {
    // Clear leftover state from previous tests
    producers.clear();
    consumers.clear();

    const producer = await connectWs("producer");
    expect(producers.size).toBe(1);

    producer.close();
    // Wait for server-side close handler
    await new Promise((r) => setTimeout(r, 100));

    expect(producers.size).toBe(0);
  });

  it("cleans up consumer set on disconnect", async () => {
    producers.clear();
    consumers.clear();

    const consumer = await connectWs();
    expect(consumers.size).toBe(1);

    consumer.close();
    await new Promise((r) => setTimeout(r, 100));

    expect(consumers.size).toBe(0);
  });

  it("consumer receives nothing when no producer is connected", async () => {
    const consumer = await connectWs();

    let received = false;
    consumer.on("message", () => {
      received = true;
    });

    await new Promise((r) => setTimeout(r, 100));
    expect(received).toBe(false);

    consumer.close();
  });
});
