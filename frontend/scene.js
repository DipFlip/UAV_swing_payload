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

function createDroneSystem(scene, bodyColor, weightColor, ropeColor, propColor, forceColor) {
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
    const shaftMat = new THREE.MeshStandardMaterial({ color: forceColor });
    const shaft = new THREE.Mesh(shaftGeo, shaftMat);
    arrowGroup.add(shaft);
    const headGeo = new THREE.ConeGeometry(0.2, 0.5, 8);
    headGeo.translate(0, 0.25, 0); // pivot at base of cone
    const headMat = new THREE.MeshStandardMaterial({ color: forceColor });
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
    scene.fog = new THREE.Fog(0x9999aa, 80, 160);

    // Camera
    const camera = new THREE.PerspectiveCamera(
        60, window.innerWidth / window.innerHeight, 0.1, 200
    );
    camera.position.set(18, 14, 18);
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
    dirLight.shadow.camera.left = -20;
    dirLight.shadow.camera.right = 20;
    dirLight.shadow.camera.top = 20;
    dirLight.shadow.camera.bottom = -20;
    dirLight.shadow.camera.near = 1;
    dirLight.shadow.camera.far = 50;
    dirLight.shadow.mapSize.width = 2048;
    dirLight.shadow.mapSize.height = 2048;
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

    // --- Drone A (blue) — dark body, bright payload ---
    const lqr = createDroneSystem(scene, 0x1155bb, 0x4499ff, 0x2288ff, 0x44ff44, 0x66bbff);

    // --- Drone B (orange) — dark body, bright payload ---
    const pid = createDroneSystem(scene, 0xcc5500, 0xffaa33, 0xff8800, 0xff6644, 0xffbb44);

    // --- Wind streamline particles (cylinder meshes for visible thickness) ---
    const WIND_PARTICLE_COUNT = 40;
    const WIND_FIELD_SIZE = 20;
    const WIND_LINE_LEN = 1.5;
    const WIND_RADIUS = 0.04;

    const windCylGeo = new THREE.CylinderGeometry(1, 1, 1, 4);
    const windCylMat = new THREE.MeshBasicMaterial({
        color: 0xaaddff,
        transparent: true,
        opacity: 0.35,
        depthWrite: false,
    });
    const windMesh = new THREE.InstancedMesh(windCylGeo, windCylMat, WIND_PARTICLE_COUNT);
    windMesh.count = 0;
    windMesh.frustumCulled = false;
    windMesh.visible = false;
    scene.add(windMesh);

    const windParticles = [];
    for (let i = 0; i < WIND_PARTICLE_COUNT; i++) {
        windParticles.push({
            x: (Math.random() - 0.5) * WIND_FIELD_SIZE * 2,
            y: 1 + Math.random() * 10,
            z: (Math.random() - 0.5) * WIND_FIELD_SIZE * 2,
        });
    }

    const _wMat = new THREE.Matrix4();
    const _wMid = new THREE.Vector3();
    const _wDir = new THREE.Vector3();
    const _wUp = new THREE.Vector3(0, 1, 0);
    const _wQuat = new THREE.Quaternion();
    const _wScale = new THREE.Vector3();

    function updateWind(windStrength, windDirRad, dt) {
        if (windStrength < 0.1) {
            windMesh.visible = false;
            return;
        }
        windMesh.visible = true;

        // Wind direction in Three.js coords: physics (cos,sin,0) → Three.js (cos,0,sin)
        const dx = Math.cos(windDirRad);
        const dz = Math.sin(windDirRad);
        const speed = windStrength * 0.3;

        for (let i = 0; i < WIND_PARTICLE_COUNT; i++) {
            const p = windParticles[i];
            p.x += dx * speed * dt;
            p.z += dz * speed * dt;

            // Wrap particles to upwind side
            const dotPos = p.x * dx + p.z * dz;
            if (dotPos > WIND_FIELD_SIZE) {
                p.x -= dx * WIND_FIELD_SIZE * 2;
                p.z -= dz * WIND_FIELD_SIZE * 2;
                p.x += (-dz) * (Math.random() - 0.5) * WIND_FIELD_SIZE * 2;
                p.z += dx * (Math.random() - 0.5) * WIND_FIELD_SIZE * 2;
                p.y = 1 + Math.random() * 10;
            }

            // Position cylinder from tail to head
            const tailLen = WIND_LINE_LEN * Math.min(windStrength / 10, 1.5);
            const tx = p.x - dx * tailLen;
            const tz = p.z - dz * tailLen;
            _wMid.set((tx + p.x) / 2, p.y, (tz + p.z) / 2);
            _wDir.set(p.x - tx, 0, p.z - tz);
            const len = _wDir.length();
            if (len > 0.001) {
                _wDir.divideScalar(len);
                _wQuat.setFromUnitVectors(_wUp, _wDir);
            }
            _wScale.set(WIND_RADIUS, len, WIND_RADIUS);
            _wMat.compose(_wMid, _wQuat, _wScale);
            windMesh.setMatrixAt(i, _wMat);
        }
        windMesh.count = WIND_PARTICLE_COUNT;
        windMesh.instanceMatrix.needsUpdate = true;
    }

    // --- Goal marker (final destination) ---
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

    // --- Reference marker (animated smoother position) ---
    const refGeo = new THREE.SphereGeometry(0.25, 12, 12);
    const refMat = new THREE.MeshStandardMaterial({
        color: 0xffcc00,
        transparent: true,
        opacity: 0.8,
    });
    const refMarker = new THREE.Mesh(refGeo, refMat);
    refMarker.position.set(0, 0, 0);
    scene.add(refMarker);

    // --- Trails (instanced cylinder segments for visible thickness) ---
    const TRAIL_MAX = 200;
    const trailCylGeo = new THREE.CylinderGeometry(1, 1, 1, 5);

    function createTrail(color, maxPoints) {
        const mat = new THREE.MeshBasicMaterial({
            color, transparent: true, opacity: 0.6, depthWrite: false,
        });
        const mesh = new THREE.InstancedMesh(trailCylGeo, mat, maxPoints - 1);
        mesh.count = 0;
        mesh.frustumCulled = false;
        scene.add(mesh);
        return { mesh, points: [], maxPoints };
    }

    const trails = {
        lqrDrone:  createTrail(0x1155bb, TRAIL_MAX),
        lqrWeight: createTrail(0x4499ff, TRAIL_MAX),
        pidDrone:  createTrail(0xcc5500, TRAIL_MAX),
        pidWeight: createTrail(0xffaa33, TRAIL_MAX),
        goal:      createTrail(0x00ff88, TRAIL_MAX),
    };

    // --- Pattern preview line ---
    const previewLineMat = new THREE.LineDashedMaterial({
        color: 0x00ff88,
        transparent: true,
        opacity: 0.5,
        dashSize: 0.3,
        gapSize: 0.15,
        linewidth: 1,
    });
    const previewLineGeo = new THREE.BufferGeometry();
    const previewLine = new THREE.Line(previewLineGeo, previewLineMat);
    previewLine.visible = false;
    scene.add(previewLine);

    function updatePatternPreview(points) {
        // points: array of [x,y,z] in physics coords
        if (!points || points.length < 2) {
            previewLine.visible = false;
            return;
        }
        const verts = new Float32Array(points.length * 3);
        for (let i = 0; i < points.length; i++) {
            const p = physicsToThree(points[i][0], points[i][1], points[i][2]);
            verts[i * 3] = p.x;
            verts[i * 3 + 1] = p.y;
            verts[i * 3 + 2] = p.z;
        }
        previewLineGeo.setAttribute('position', new THREE.BufferAttribute(verts, 3));
        previewLineGeo.computeBoundingSphere();
        previewLine.computeLineDistances();
        previewLine.visible = true;
    }

    function hidePatternPreview() {
        previewLine.visible = false;
    }

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
        const spriteMat = new THREE.SpriteMaterial({
            map: texture,
            transparent: true,
            depthTest: false,
            depthWrite: false,
        });
        const sprite = new THREE.Sprite(spriteMat);
        sprite.scale.set(1.6, 0.5, 1);
        sprite.renderOrder = 999;
        scene.add(sprite);
        return sprite;
    }

    const lqrLabel = makeLabel('LQR', '#4499ff');
    const pidLabel = makeLabel('PID', '#ff8800');

    function updateLabelText(sprite, text, color) {
        const canvas2d = document.createElement('canvas');
        canvas2d.width = 256;
        canvas2d.height = 40;
        const ctx = canvas2d.getContext('2d');
        ctx.font = 'bold 24px sans-serif';
        ctx.fillStyle = color;
        ctx.textAlign = 'center';
        ctx.fillText(text, 128, 28);
        if (sprite.material.map) sprite.material.map.dispose();
        sprite.material.map = new THREE.CanvasTexture(canvas2d);
        sprite.material.needsUpdate = true;
    }

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
        goalMarker, refMarker,
        lqrLabel, pidLabel,
        updateLabelText,
        trails,
        updatePatternPreview,
        hidePatternPreview,
        updateWind,
        setOnAnimate(fn) { onAnimate = fn; },
    };
}
