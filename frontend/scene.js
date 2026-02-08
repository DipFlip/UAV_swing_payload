/**
 * Three.js scene setup: dual drone systems, rope, weight, goal marker, ground.
 *
 * Physics coordinate system: z-up
 * Three.js coordinate system: y-up
 * Mapping: physics (px, py, pz) -> Three.js (px, pz, py)
 */

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

export function physicsToThree(px, py, pz) {
    return { x: px, y: pz, z: py };
}

function createDroneSystem(scene, bodyColor, weightColor, ropeColor, propColor) {
    const group = new THREE.Group();

    // Body
    const bodyGeo = new THREE.BoxGeometry(0.6, 0.2, 0.6);
    const bodyMat = new THREE.MeshStandardMaterial({ color: bodyColor });
    const body = new THREE.Mesh(bodyGeo, bodyMat);
    body.castShadow = true;
    group.add(body);

    // Arms
    const armGeo = new THREE.BoxGeometry(1.8, 0.05, 0.08);
    const armMat = new THREE.MeshStandardMaterial({ color: 0x888888 });
    const arm1 = new THREE.Mesh(armGeo, armMat);
    group.add(arm1);
    const arm2 = new THREE.Mesh(armGeo, armMat);
    arm2.rotation.y = Math.PI / 2;
    group.add(arm2);

    // Propeller discs
    const propGeo = new THREE.CylinderGeometry(0.25, 0.25, 0.02, 16);
    const propMat = new THREE.MeshStandardMaterial({ color: propColor, transparent: true, opacity: 0.5 });
    [[0.8, 0.1, 0], [-0.8, 0.1, 0], [0, 0.1, 0.8], [0, 0.1, -0.8]].forEach(pos => {
        const prop = new THREE.Mesh(propGeo, propMat);
        prop.position.set(...pos);
        group.add(prop);
    });

    group.position.set(0, 4, 0);
    scene.add(group);

    // Force arrow (cylinder shaft + cone head for visible thickness)
    const arrowGroup = new THREE.Group();
    const shaftGeo = new THREE.CylinderGeometry(0.06, 0.06, 1, 8);
    shaftGeo.translate(0, 0.5, 0); // pivot at base
    const shaftMat = new THREE.MeshStandardMaterial({ color: 0xff4444 });
    const shaft = new THREE.Mesh(shaftGeo, shaftMat);
    arrowGroup.add(shaft);
    const headGeo = new THREE.ConeGeometry(0.2, 0.5, 8);
    headGeo.translate(0, 0.25, 0); // pivot at base of cone
    const headMat = new THREE.MeshStandardMaterial({ color: 0xff4444 });
    const head = new THREE.Mesh(headGeo, headMat);
    arrowGroup.add(head);
    arrowGroup.visible = false;
    scene.add(arrowGroup);
    const forceArrow = { group: arrowGroup, shaft, head };

    // Rope (cylinder mesh for visible thickness)
    const ropeGeo = new THREE.CylinderGeometry(0.03, 0.03, 1, 6);
    const ropeMat = new THREE.MeshStandardMaterial({ color: ropeColor });
    const rope = new THREE.Mesh(ropeGeo, ropeMat);
    scene.add(rope);

    // Weight
    const weightGeo = new THREE.SphereGeometry(0.3, 24, 24);
    const weightMat = new THREE.MeshStandardMaterial({ color: weightColor });
    const weight = new THREE.Mesh(weightGeo, weightMat);
    weight.castShadow = true;
    scene.add(weight);

    return { droneGroup: group, rope, weight, forceArrow };
}

export function createScene(canvas) {
    const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
    renderer.setSize(window.innerWidth, window.innerHeight);
    renderer.setPixelRatio(window.devicePixelRatio);
    renderer.shadowMap.enabled = true;

    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x9999aa);
    scene.fog = new THREE.Fog(0x9999aa, 40, 80);

    // Camera
    const camera = new THREE.PerspectiveCamera(
        60, window.innerWidth / window.innerHeight, 0.1, 200
    );
    camera.position.set(12, 10, 12);
    camera.lookAt(0, 4, 0);

    // Controls
    const controls = new OrbitControls(camera, canvas);
    controls.target.set(0, 4, 0);
    controls.enableDamping = true;
    controls.dampingFactor = 0.1;
    controls.update();

    // Lighting
    const ambient = new THREE.AmbientLight(0xffffff, 0.6);
    scene.add(ambient);

    const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
    dirLight.position.set(10, 20, 10);
    dirLight.castShadow = true;
    scene.add(dirLight);

    // Ground grid (raised slightly to avoid z-fighting with ground plane)
    const gridHelper = new THREE.GridHelper(40, 20, 0xaaaaaa, 0x999999);
    gridHelper.position.y = 0.01;
    scene.add(gridHelper);

    // Ground plane
    const groundGeo = new THREE.PlaneGeometry(40, 40);
    const groundMat = new THREE.MeshStandardMaterial({ color: 0x888899, roughness: 0.9 });
    const ground = new THREE.Mesh(groundGeo, groundMat);
    ground.rotation.x = -Math.PI / 2;
    ground.receiveShadow = true;
    scene.add(ground);

    // --- LQR system (blue drone, red weight) ---
    const lqr = createDroneSystem(scene, 0x2288ff, 0xff3333, 0xcccccc, 0x44ff44);

    // --- PID system (orange drone, yellow weight) ---
    const pid = createDroneSystem(scene, 0xff8800, 0xffcc00, 0xffaa66, 0xff6644);

    // --- Goal marker ---
    const goalGeo = new THREE.SphereGeometry(0.4, 16, 16);
    const goalMat = new THREE.MeshStandardMaterial({
        color: 0x00ff88,
        wireframe: true,
        transparent: true,
        opacity: 0.6,
    });
    const goalMarker = new THREE.Mesh(goalGeo, goalMat);
    goalMarker.position.set(0, 0, 0);
    scene.add(goalMarker);

    // --- Trails ---
    function createTrail(color, maxPoints) {
        const positions = new Float32Array(maxPoints * 3);
        const geo = new THREE.BufferGeometry();
        geo.setAttribute('position', new THREE.BufferAttribute(positions, 3));
        geo.setDrawRange(0, 0);
        const mat = new THREE.LineBasicMaterial({ color, transparent: true, opacity: 0.6, linewidth: 2 });
        const line = new THREE.Line(geo, mat);
        scene.add(line);
        return { line, head: 0, count: 0, maxPoints };
    }

    const TRAIL_MAX = 200; // enough points for ~2s of trail at sim rate
    const trails = {
        lqrDrone:  createTrail(0x4499ff, TRAIL_MAX),
        lqrWeight: createTrail(0xff3333, TRAIL_MAX),
        pidDrone:  createTrail(0xff8800, TRAIL_MAX),
        pidWeight: createTrail(0xffcc00, TRAIL_MAX),
        goal:      createTrail(0x00ff88, TRAIL_MAX),
    };

    // --- Labels (floating text sprites) ---
    function makeLabel(text, color) {
        const canvas2d = document.createElement('canvas');
        canvas2d.width = 128;
        canvas2d.height = 40;
        const ctx = canvas2d.getContext('2d');
        ctx.font = 'bold 24px sans-serif';
        ctx.fillStyle = color;
        ctx.textAlign = 'center';
        ctx.fillText(text, 64, 28);

        const texture = new THREE.CanvasTexture(canvas2d);
        const spriteMat = new THREE.SpriteMaterial({ map: texture, transparent: true });
        const sprite = new THREE.Sprite(spriteMat);
        sprite.scale.set(1.6, 0.5, 1);
        scene.add(sprite);
        return sprite;
    }

    const lqrLabel = makeLabel('LQR', '#4499ff');
    const pidLabel = makeLabel('PID', '#ff8800');

    // Shift camera view so drones are centered in the space above the controls panel
    function updateViewOffset() {
        const W = window.innerWidth;
        const H = window.innerHeight;
        const panel = document.getElementById('controls');
        const ph = panel ? panel.getBoundingClientRect().height + 10 : 0;
        camera.aspect = W / H;
        if (ph > 20) {
            camera.setViewOffset(W, H + ph, 0, ph, W, H);
        } else {
            camera.clearViewOffset();
        }
        camera.updateProjectionMatrix();
        renderer.setSize(W, H);
    }

    // Handle resize
    updateViewOffset();
    window.addEventListener('resize', updateViewOffset);

    // Animation loop — callback receives wall-clock delta in seconds
    let onAnimate = null;
    let lastTime = 0;

    function animate(now) {
        requestAnimationFrame(animate);
        const dt = lastTime ? (now - lastTime) / 1000 : 0;
        lastTime = now;
        if (onAnimate) onAnimate(dt);
        controls.update();
        renderer.render(scene, camera);
    }
    requestAnimationFrame(animate);

    return {
        scene, camera, renderer, controls,
        lqr, pid,
        goalMarker,
        lqrLabel, pidLabel,
        trails,
        setOnAnimate(fn) { onAnimate = fn; },
    };
}
