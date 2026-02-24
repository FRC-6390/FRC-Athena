<script lang="ts">
  import { onMount } from 'svelte';
  import * as THREE from 'three';

  type Props = {
    rollDeg: number;
    pitchDeg: number;
    yawDeg: number;
    className?: string;
  };

  let { rollDeg, pitchDeg, yawDeg, className = '' }: Props = $props();

  let hostEl = $state<HTMLDivElement | null>(null);
  let webglUnavailable = $state(false);

  let renderer: THREE.WebGLRenderer | null = null;
  let scene: THREE.Scene | null = null;
  let camera: THREE.PerspectiveCamera | null = null;
  let imuBody: THREE.Group | null = null;
  let resizeObserver: ResizeObserver | null = null;

  function renderNow() {
    if (!renderer || !scene || !camera) return;
    renderer.render(scene, camera);
  }

  function syncOrientation() {
    if (!imuBody) return;
    imuBody.rotation.order = 'ZXY';
    imuBody.rotation.x = THREE.MathUtils.degToRad(pitchDeg);
    imuBody.rotation.y = THREE.MathUtils.degToRad(rollDeg);
    imuBody.rotation.z = THREE.MathUtils.degToRad(yawDeg);
  }

  function resizeRenderer() {
    if (!hostEl || !renderer || !camera) return;
    const width = Math.max(1, Math.floor(hostEl.clientWidth));
    const height = Math.max(1, Math.floor(hostEl.clientHeight));
    renderer.setSize(width, height, false);
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
    renderNow();
  }

  function disposeSceneGraph(root: THREE.Object3D) {
    root.traverse((node) => {
      const mesh = node as THREE.Mesh;
      if (mesh.geometry) {
        mesh.geometry.dispose();
      }
      const material = mesh.material as THREE.Material | THREE.Material[] | undefined;
      if (Array.isArray(material)) {
        for (const item of material) item.dispose();
      } else if (material) {
        material.dispose();
      }
    });
  }

  $effect(() => {
    syncOrientation();
    renderNow();
  });

  onMount(() => {
    if (!hostEl) return;

    try {
      renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true, powerPreference: 'high-performance' });
    } catch (_error) {
      webglUnavailable = true;
      return;
    }

    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    renderer.setClearColor(0x000000, 0);
    renderer.outputColorSpace = THREE.SRGBColorSpace;
    hostEl.appendChild(renderer.domElement);

    scene = new THREE.Scene();
    camera = new THREE.PerspectiveCamera(44, 1, 0.1, 100);
    camera.position.set(2.8, 2.1, 2.8);
    camera.lookAt(0, 0, 0);

    const ambient = new THREE.AmbientLight(0xffffff, 0.56);
    scene.add(ambient);

    const keyLight = new THREE.DirectionalLight(0xffffff, 0.9);
    keyLight.position.set(2.8, 3.4, 2.2);
    scene.add(keyLight);

    const fillLight = new THREE.DirectionalLight(0x93c5fd, 0.34);
    fillLight.position.set(-2.2, 1.4, -2.8);
    scene.add(fillLight);

    const floor = new THREE.GridHelper(3.5, 10, 0x334155, 0x1e293b);
    floor.position.y = -0.55;
    floor.material.transparent = true;
    floor.material.opacity = 0.45;
    scene.add(floor);

    const axis = new THREE.AxesHelper(1.2);
    scene.add(axis);

    imuBody = new THREE.Group();

    const bodyGeometry = new THREE.BoxGeometry(1.08, 0.32, 0.86);
    const bodyMaterials = [
      new THREE.MeshStandardMaterial({ color: 0x7f1d1d, metalness: 0.3, roughness: 0.45 }),
      new THREE.MeshStandardMaterial({ color: 0x1f2937, metalness: 0.24, roughness: 0.52 }),
      new THREE.MeshStandardMaterial({ color: 0x334155, metalness: 0.26, roughness: 0.5 }),
      new THREE.MeshStandardMaterial({ color: 0x111827, metalness: 0.2, roughness: 0.58 }),
      new THREE.MeshStandardMaterial({ color: 0xdc2626, metalness: 0.3, roughness: 0.36 }),
      new THREE.MeshStandardMaterial({ color: 0x0f172a, metalness: 0.24, roughness: 0.54 })
    ];

    const bodyMesh = new THREE.Mesh(bodyGeometry, bodyMaterials);
    bodyMesh.castShadow = false;
    bodyMesh.receiveShadow = false;
    imuBody.add(bodyMesh);

    const edge = new THREE.LineSegments(
      new THREE.EdgesGeometry(bodyGeometry),
      new THREE.LineBasicMaterial({ color: 0x94a3b8, transparent: true, opacity: 0.85 })
    );
    imuBody.add(edge);

    const forwardArrow = new THREE.ArrowHelper(
      new THREE.Vector3(1, 0, 0),
      new THREE.Vector3(0, 0.01, 0),
      0.95,
      0xef4444,
      0.22,
      0.12
    );
    imuBody.add(forwardArrow);

    scene.add(imuBody);

    syncOrientation();

    resizeObserver = new ResizeObserver(() => resizeRenderer());
    resizeObserver.observe(hostEl);
    resizeRenderer();

    return () => {
      resizeObserver?.disconnect();
      resizeObserver = null;

      if (scene) {
        disposeSceneGraph(scene);
      }

      if (renderer) {
        renderer.dispose();
        const canvas = renderer.domElement;
        if (canvas.parentElement === hostEl) {
          hostEl.removeChild(canvas);
        }
      }

      renderer = null;
      scene = null;
      camera = null;
      imuBody = null;
    };
  });
</script>

<div class={`imu-3d-host ${className}`.trim()} bind:this={hostEl}>
  {#if webglUnavailable}
    <div class="fallback">WebGL unavailable</div>
  {/if}
</div>

<style>
  .imu-3d-host {
    position: relative;
    width: 100%;
    height: 100%;
    min-height: 7.5rem;
    border: 1px solid var(--border-subtle);
    border-radius: 8px;
    background: radial-gradient(circle at 50% 30%, #1a2436, #101827 74%);
    overflow: hidden;
  }

  .imu-3d-host :global(canvas) {
    width: 100%;
    height: 100%;
    display: block;
  }

  .fallback {
    position: absolute;
    inset: 0;
    display: grid;
    place-items: center;
    font-size: 0.68rem;
    color: var(--text-soft);
    text-transform: uppercase;
    letter-spacing: 0.04em;
  }
</style>
