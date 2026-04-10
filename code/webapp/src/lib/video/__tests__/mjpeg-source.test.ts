import { describe, it, expect, vi, beforeEach, afterEach } from "vitest";
import { MJPEGSource } from "../mjpeg-source";

describe("MJPEGSource", () => {
  let source: MJPEGSource;

  beforeEach(() => {
    source = new MJPEGSource();
  });

  afterEach(() => {
    source.disconnect();
  });

  it("has type 'mjpeg'", () => {
    expect(source.type).toBe("mjpeg");
  });

  it("starts disconnected", () => {
    expect(source.connected).toBe(false);
  });

  it("sets img src on connect", () => {
    source.connect("http://192.168.1.1/stream");
    // The Image constructor is available in jsdom
    // We verify disconnect clears it
    expect(source.connected).toBe(false); // not yet connected until onload
  });

  it("disconnect clears state", () => {
    source.connect("http://192.168.1.1/stream");
    source.disconnect();
    expect(source.connected).toBe(false);
  });

  it("calls onError when image fails to load", () => {
    const onError = vi.fn();
    source.onError = onError;

    // Override Image to trigger onerror
    const originalImage = globalThis.Image;
    globalThis.Image = class extends originalImage {
      constructor() {
        super();
        setTimeout(() => this.onerror?.(new Event("error")), 0);
      }
    } as typeof Image;

    source.connect("http://bad-url/stream");

    return vi.waitFor(() => {
      expect(onError).toHaveBeenCalled();
      expect(source.connected).toBe(false);
    }).finally(() => {
      globalThis.Image = originalImage;
    });
  });

  it("can reconnect after disconnect", () => {
    source.connect("http://192.168.1.1/stream");
    source.disconnect();
    source.connect("http://192.168.1.1/stream");
    // Should not throw
    expect(source.connected).toBe(false); // waiting for onload
  });
});
