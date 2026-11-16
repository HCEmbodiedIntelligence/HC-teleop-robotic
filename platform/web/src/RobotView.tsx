import { useEffect, useRef } from "react";
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import URDFLoader from "urdf-loader";

export default function RobotView({
  config,
  control,
}: {
  config: any;
  control: any;
}) {
  const host = useRef<HTMLDivElement>(null),
    robot = useRef<any>(null);
  useEffect(() => {
    if (!host.current || !config?.urdf) return;
    const element = host.current,
      scene = new THREE.Scene();
    delete element.dataset.error;
    scene.background = new THREE.Color("#121c28");
    const camera = new THREE.PerspectiveCamera(
      45,
      element.clientWidth / 400,
      0.01,
      100,
    );
    camera.position.set(2, -2, 1.8);
    camera.up.set(0, 0, 1);
    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
    renderer.setSize(element.clientWidth, 400);
    element.appendChild(renderer.domElement);
    const orbit = new OrbitControls(camera, renderer.domElement);
    orbit.target.set(0, 0, 1);
    orbit.update();
    scene.add(new THREE.HemisphereLight(0xffffff, 0x334455, 2));
    const light = new THREE.DirectionalLight(0xffffff, 2);
    light.position.set(2, -3, 5);
    scene.add(light);
    const grid = new THREE.GridHelper(5, 20, 0x506070, 0x243445);
    grid.rotation.x = Math.PI / 2;
    scene.add(grid, new THREE.AxesHelper(0.3));
    const loader = new URDFLoader();
    loader.packages = () => "/api/model";
    let disposed = false;
    const relative = config.package_root
      ? config.urdf.replace(config.package_root.replace(/\/+$/, "") + "/", "")
      : config.urdf;
    loader.load(
      "/api/model/" + relative,
      (value: any) => {
        if (disposed) return;
        robot.current = value;
        scene.add(value);
      },
      undefined,
      () => {
        element.dataset.error = "模型资源加载失败";
      },
    );
    let frame = 0;
    const render = () => {
      frame = requestAnimationFrame(render);
      orbit.update();
      renderer.render(scene, camera);
    };
    render();
    const resize = new ResizeObserver(() => {
      renderer.setSize(element.clientWidth, 400);
      camera.aspect = element.clientWidth / 400;
      camera.updateProjectionMatrix();
    });
    resize.observe(element);
    return () => {
      disposed = true;
      cancelAnimationFrame(frame);
      resize.disconnect();
      orbit.dispose();
      scene.traverse((item: any) => {
        item.geometry?.dispose();
        if (Array.isArray(item.material))
          item.material.forEach((m: any) => m.dispose());
        else item.material?.dispose();
      });
      renderer.dispose();
      renderer.domElement.remove();
      robot.current = null;
    };
  }, [config?.config_hash]);
  useEffect(() => {
    const names = control?.joint_names || config?.joint_names || [],
      positions =
        control?.feedback_positions ||
        control?.positions ||
        control?.feedback?.positions ||
        [];
    if (robot.current)
      names.forEach((name: string, i: number) => {
        if (Number.isFinite(positions[i]))
          robot.current.setJointValue(name, positions[i]);
      });
  }, [control]);
  return <div ref={host} className="robot-view" aria-label="机器人三维模型" />;
}
