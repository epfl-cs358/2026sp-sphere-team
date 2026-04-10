import { describe, it, expect, vi, beforeEach, afterEach } from "vitest";
import { MockSource } from "../mock-source";

describe("MockSource", () => {
  let source: MockSource;

  beforeEach(() => {
    source = new MockSource();
  });

  afterEach(() => {
    source.disconnect();
  });

  it("has type 'mock'", () => {
    expect(source.type).toBe("mock");
  });

  it("starts disconnected", () => {
    expect(source.connected).toBe(false);
  });

  it("connects and sets connected to true", () => {
    source.connect("");
    expect(source.connected).toBe(true);
  });

  it("disconnects and sets connected to false", () => {
    source.connect("");
    source.disconnect();
    expect(source.connected).toBe(false);
  });

  it("creates Image elements when connected", () => {
    const origImage = globalThis.Image;
    const created: HTMLImageElement[] = [];
    globalThis.Image = class extends origImage {
      constructor() {
        super();
        created.push(this);
      }
    } as typeof Image;

    source.connect("");
    expect(created.length).toBeGreaterThan(0);

    globalThis.Image = origImage;
  });

  it("stops rendering after disconnect", async () => {
    const cancelSpy = vi.spyOn(globalThis, "cancelAnimationFrame");
    source.connect("");
    source.disconnect();
    expect(cancelSpy).toHaveBeenCalled();
    cancelSpy.mockRestore();
  });

  it("can reconnect after disconnect", () => {
    source.connect("");
    source.disconnect();
    expect(source.connected).toBe(false);

    source.connect("");
    expect(source.connected).toBe(true);
  });
});
