/**
 * Minimal WebGL2 line renderer for the Analysis plots. Draws pre-normalised clip-space line strips; no
 * dependencies. One program, one buffer, one draw call per series.
 */
const VERTEX = `#version 300 es
in vec2 a_position;
void main() { gl_Position = vec4(a_position, 0.0, 1.0); }`;

const FRAGMENT = `#version 300 es
precision mediump float;
uniform vec4 u_color;
out vec4 outColor;
void main() { outColor = u_color; }`;

export class WebGLLinePlot {
  private readonly gl: WebGL2RenderingContext;
  private readonly program: WebGLProgram;
  private readonly buffer: WebGLBuffer;
  private readonly colorLocation: WebGLUniformLocation;

  constructor(readonly canvas: HTMLCanvasElement) {
    const gl = canvas.getContext('webgl2', { antialias: true, premultipliedAlpha: true });
    if (!gl) throw new Error('WebGL2 is not available');
    this.gl = gl;
    this.program = link(gl, VERTEX, FRAGMENT);
    const buffer = gl.createBuffer();
    if (!buffer) throw new Error('could not allocate a WebGL buffer');
    this.buffer = buffer;
    const location = gl.getUniformLocation(this.program, 'u_color');
    if (!location) throw new Error('missing u_color uniform');
    this.colorLocation = location;
  }

  /** Matches the backing store to the displayed size (device pixels). Returns the pixel size. */
  resize(): { width: number; height: number } {
    const dpr = Math.min(2, window.devicePixelRatio || 1);
    const width = Math.max(1, Math.round(this.canvas.clientWidth * dpr));
    const height = Math.max(1, Math.round(this.canvas.clientHeight * dpr));
    if (this.canvas.width !== width || this.canvas.height !== height) {
      this.canvas.width = width;
      this.canvas.height = height;
    }
    return { width, height };
  }

  /** Clears and draws each series as a line strip. `color` is RGBA in 0..1. */
  draw(series: { vertices: Float32Array; color: [number, number, number, number] }[]): void {
    const gl = this.gl;
    gl.viewport(0, 0, this.canvas.width, this.canvas.height);
    gl.clearColor(0, 0, 0, 0);
    gl.clear(gl.COLOR_BUFFER_BIT);
    gl.useProgram(this.program);
    gl.bindBuffer(gl.ARRAY_BUFFER, this.buffer);
    const location = gl.getAttribLocation(this.program, 'a_position');
    gl.enableVertexAttribArray(location);
    gl.vertexAttribPointer(location, 2, gl.FLOAT, false, 0, 0);
    for (const s of series) {
      if (s.vertices.length < 4) continue;
      gl.bufferData(gl.ARRAY_BUFFER, s.vertices, gl.DYNAMIC_DRAW);
      gl.uniform4f(this.colorLocation, s.color[0], s.color[1], s.color[2], s.color[3]);
      gl.drawArrays(gl.LINE_STRIP, 0, s.vertices.length / 2);
    }
  }

  dispose(): void {
    this.gl.deleteBuffer(this.buffer);
    this.gl.deleteProgram(this.program);
  }
}

function compile(gl: WebGL2RenderingContext, type: number, source: string): WebGLShader {
  const shader = gl.createShader(type);
  if (!shader) throw new Error('could not create shader');
  gl.shaderSource(shader, source);
  gl.compileShader(shader);
  if (!gl.getShaderParameter(shader, gl.COMPILE_STATUS)) {
    const log = gl.getShaderInfoLog(shader);
    gl.deleteShader(shader);
    throw new Error(`shader compile failed: ${log}`);
  }
  return shader;
}

function link(gl: WebGL2RenderingContext, vertex: string, fragment: string): WebGLProgram {
  const program = gl.createProgram();
  if (!program) throw new Error('could not create program');
  gl.attachShader(program, compile(gl, gl.VERTEX_SHADER, vertex));
  gl.attachShader(program, compile(gl, gl.FRAGMENT_SHADER, fragment));
  gl.linkProgram(program);
  if (!gl.getProgramParameter(program, gl.LINK_STATUS)) {
    throw new Error(`program link failed: ${gl.getProgramInfoLog(program)}`);
  }
  return program;
}
