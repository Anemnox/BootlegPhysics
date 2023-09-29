import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { DragControls } from "three/addons/controls/DragControls.js";

const Vector3 = THREE.Vector3;
const Euler = THREE.Euler;

Euler.prototype.add = function (euler) {
  this.x += euler.x;
  this.y += euler.y;
  this.z += euler.z;
  return this;
};

Euler.prototype.scale = function (scalar) {
  this.x *= scalar;
  this.y *= scalar;
  this.z *= scalar;
  return this;
};

Euler.prototype.lengthSq = function () {
  return this.x * this.x + this.y * this.y + this.z * this.z;
};

Vector3.prototype.toString = function () {
  return "{" + this.x + ", " + this.y + ", " + this.z + "}";
};

/**
 * Brute Force Object Updates
 * @param {} objects List of objects to update
 * @param {number} ms Delta T between updates
 * @param {} params Set of parameters related to the update
 */
function updatePhysics(objects, ms, params) {
  ms *= 0.001;

  for (let i = 0; i < objects.length; i++) {
    const obj1 = objects[i];
    let velVector = obj1.velocity.clone();
    let rotVector = obj1.rotationalVelocity.clone();

    if (obj1.needsUpdate) {
      if (params.worldForces) {
        params.worldForces.forEach((force) => {
          obj1.applyForce(force);
        });
      }

      obj1.updateVelocity(ms);

      for (let j = 0; j < objects.length; j++) {
        if (i === j) continue;
        const obj2 = objects[j];
        let sphere1 = new THREE.Sphere();
        let sphere2 = new THREE.Sphere();
        sphere1.copy(obj1.geoBound.boundingSphere);
        sphere2.copy(obj2.geoBound.boundingSphere);

        sphere1.center = obj1.position.clone().add(velVector);
        sphere2.center = obj2.position;
        if (sphere1.intersectsSphere(sphere2)) {
          obj1.graphicsObject.material.color.setHex(0xff0000);

          let [vel, rot] = constrainCollisions(
            velVector,
            rotVector,
            obj1,
            obj2,
          );
          if (vel.lengthSq() < velVector.lengthSq()) {
            velVector = vel;
          }
          if (rot.lengthSq() < rotVector.lengthSq()) {
            rotVector = rot;
          }
        }
      }
    }
    obj1.rotate(rotVector);
    obj1.move(velVector);
  }
}

function constrainCollisions(vel, objRot, obj1, obj2) {
  let collisionTime = 2;
  let normalForce = new Vector3(0, 0, 0);
  let contactPoint = null;
  let geo1 = obj1.geoBound;
  let geo2 = obj2.geoBound;
  let constrainedVelocity = vel.clone();
  let constrainedRotation = objRot.clone();

  geo1.vertices.forEach((vertex, i) => {
    let velocity = vel.clone();
    let point = vertex.clone().applyEuler(obj1.rotation);

    let isRotating = obj1.rotationalVelocity.lengthSq() !== 0;

    if (isRotating) {
      velocity = point
        .clone()
        .sub(vertex.clone().applyEuler(obj1.rotation.clone().add(objRot)))
        .add(vel);
    }

    point.add(obj1.position);

    geo2.faces.forEach((face, i) => {
      let faceNormal = face.normal.clone().applyEuler(obj2.rotation);
      if (faceNormal.dot(velocity) <= 0) {
        let facePoint = geo2.vertices[face.a]
          .clone()
          .applyEuler(obj2.rotation)
          .add(obj2.position);
        let time = timeOfIntersectPlaneLine(
          velocity,
          point,
          faceNormal,
          facePoint,
        );
        //debugger;
        if (time >= -0.01 && time < 1.1) {
          let a = geo2.vertices[face.a].clone().applyEuler(obj2.rotation);
          let b = geo2.vertices[face.b].clone().applyEuler(obj2.rotation);
          let c = geo2.vertices[face.c].clone().applyEuler(obj2.rotation);
          let relPointVect = new Vector3(
            velocity.x * time + point.x,
            velocity.y * time + point.y,
            velocity.z * time + point.z,
          );
          let p = relPointVect.clone().sub(obj2.position);
          if (isInsideTriangle(p, a, b, c)) {
            if (time > 1) time = 1;

            if (time < collisionTime) {
              //debugger;
              normalForce = faceNormal.multiplyScalar(-1);
              contactPoint = relPointVect;
              constrainedVelocity = velocity.clone().multiplyScalar(time);
            }
          }
        }
      }
    });
  });

  geo2.vertices.forEach((vertex, i) => {
    let velocity = vel.clone().multiplyScalar(-1);
    geo1.faces.forEach((face, i) => {
      let point = vertex.clone().applyEuler(obj2.rotation).add(obj2.position);
      let faceNormal = face.normal.clone().applyEuler(obj1.rotation);

      if (faceNormal.dot(velocity) <= 0) {
        let facePoint = geo1.vertices[face.a]
          .clone()
          .applyEuler(obj1.rotation)
          .add(obj1.position);
        let time = timeOfIntersectPlaneLine(
          velocity,
          point,
          faceNormal,
          facePoint,
        );

        if (time >= -0.01 && time <= 1.01) {
          let a = geo1.vertices[face.a].clone().applyEuler(obj1.rotation);
          let b = geo1.vertices[face.b].clone().applyEuler(obj1.rotation);
          let c = geo1.vertices[face.c].clone().applyEuler(obj1.rotation);
          let relPointVect = new Vector3(
            velocity.x * time + point.x,
            velocity.y * time + point.y,
            velocity.z * time + point.z,
          );
          let p = relPointVect.clone().sub(obj1.position);

          //debugger;
          if (isInsideTriangle(p, a, b, c)) {
            if (time > 1) time = 1;
            time = 1 - time;

            if (time < collisionTime) {
              //debugger;
              normalForce = faceNormal.multiplyScalar(-1);
              contactPoint = relPointVect;
              constrainedVelocity = velocity.clone().multiplyScalar(-1 * time);
            }
          }
        }
      }
    });
  });

  if (contactPoint !== null) {
    let vai =
      Math.cos(Math.PI - vel.angleTo(normalForce)) * vel.length() * 1000;
    let vbi =
      Math.cos(Math.PI - obj2.velocity.angleTo(normalForce)) *
      obj2.velocity.length() *
      1000;

    let vaf =
      ((obj1.mass - obj2.mass) / (obj1.mass + obj2.mass)) * vai +
      ((2 * obj2.mass) / (obj1.mass + obj2.mass)) * vbi;
    let vbf =
      ((2 * obj1.mass) / (obj1.mass + obj2.mass)) * vai +
      ((obj2.mass - obj1.mass) / (obj1.mass + obj2.mass)) * vbi;

    let nf = normalForce.normalize();
    obj1.applyForce(function () {
      return new ForceVector(
        nf.clone().multiplyScalar((vai - vaf) * obj1.mass * obj1.elasticity),
        contactPoint.clone().sub(obj1.position),
      );
    });
    obj2.applyForce(function () {
      return new ForceVector(
        nf.clone().multiplyScalar((vbi - vbf) * obj2.mass * obj2.elasticity),
        contactPoint.clone().sub(obj2.position),
      );
    });
  }

  return [constrainedVelocity, constrainedRotation];
}

/**
Add objects to the Scene

@param {GameObject} object reference to the object you would like to add.
*/
function addObject(object, objects, scene) {
  objects.push(object);
  scene.add(object.graphicsObject);
}

/**
  Remove objects from the Game Scene

  @param {GameObject} object reference to the object you would like to remove.
*/
function removeObject(object, objects, scene) {
  let index = objects.indexOf(object);
  if (index !== -1) {
    objects.splice(index, 1);
    scene.remove(object.graphicsObject);
  }
}

class GameObject {
  constructor(object3D) {
    this.graphicsObject = object3D;
    this.geoBound = object3D.geometry.clone();
    this.position = object3D.position.clone();
    this.rotation = object3D.rotation.clone();

    this.mass = 1;
    this.needsUpdate = true;

    this.forces = [];

    this.elasticity = 1;

    this.velocity = new Vector3(0, 0, 0);
    this.rotationalVelocity = new Euler(0, 0, 0);

    this.geoBound.computeBoundingSphere();
  }

  updateVelocity(ms) {
    let temp = [...this.forces];
    this.forces = [];
    temp.forEach((force) => {
      this.velocity.x += (force.force.x / this.mass) * ms;
      this.velocity.y += (force.force.y / this.mass) * ms;
      this.velocity.z += (force.force.z / this.mass) * ms;
    });

    let v = this.velocity.clone();
    let drv = this.rotationalVelocity.clone();
    v.multiplyScalar(ms);
    drv.scale(ms);
  }

  acceptFuture() {
    let pos = this.position;
    let rot = this.rotation;
    this.graphicsObject.position.set(pos.x, pos.y, pos.z);
    this.graphicsObject.rotation.set(rot.x, rot.y, rot.z);
  }

  applyForce(force) {
    try {
      let forceVector = force(this.position, this.mass);
      if (forceVector instanceof ForceVector && this.needsUpdate) {
        this.forces.push(forceVector);
      }
    } catch (e) {
      console.error(e);
    }
  }

  move(vector) {
    this.position.add(vector);
    this.acceptFuture();
  }

  rotate(euler) {
    this.rotation.add(euler);
    this.acceptFuture();
  }

  setPosition(x, y, z) {
    this.position.set(x, y, z);
    this.acceptFuture();
  }

  setRotation(x, y, z) {
    this.rotation.set(x, y, z);
    this.acceptFuture();
  }
}

class ForceVector {
  constructor(force, position) {
    this.force = force;
    this.position = position;
  }
}

/**
 *
 * @param {*} mesh a ThreeJS Mesh Object
 * @returns {Vector3[]} List of mesh verticies
 */
function getVerticiesFromMesh(mesh) {
  const verts = [];
  if (mesh.isMesh) {
    const position = mesh.geometry.attributes.position;
    const vector = new THREE.Vector3();

    for (let i = 0, l = position.count; i < l; i++) {
      vector.fromBufferAttribute(position, i);
      vector.applyMatrix4(mesh.matrixWorld);

      verts.push(vector);
    }
  }
  return verts;
}

/**
  Returns the time of intesection between a line and plane.
  One unit of time is represented by the given vector.

  @param {Vector3} lineVector a vector the represents the path of a point
  @param {Vector3} linePoint a vector that represents the point at t = 0
           for the path of the point
  @param {Vector3} planeNormal a vector that represents the normal vector
           of the plane
  @param {Vector3} planePoint a vector that represents a point on the plane.
*/
function timeOfIntersectPlaneLine(
  lineVector,
  linePoint,
  planeNormal,
  planePoint,
) {
  return (
    (-planeNormal.x * (linePoint.x - planePoint.x) -
      planeNormal.y * (linePoint.y - planePoint.y) -
      planeNormal.z * (linePoint.z - planePoint.z)) /
    (planeNormal.x * lineVector.x +
      planeNormal.y * lineVector.y +
      planeNormal.z * lineVector.z)
  );
}

/**
  Returns true or false based on if the given point is within the bounds
  of a triangle with the given coordinates. Assumes that the point is on
  the same plane as the triangle.

  @param {Vector3} point a point in 3d space
  @param {Vector3} a first vertex of the triangle
  @param {Vector3} b second vertex of the triangle
  @param {Vector3} c third vertex of the triangle
*/
function isInsideTriangle(point, a, b, c) {
  let v1 = b.clone().sub(a);
  let v2 = c.clone().sub(a);
  let v3 = point.clone().sub(a);

  let omega1 = v1.angleTo(v2);
  let omega2 = v2.angleTo(v3);
  let omega3 = v1.angleTo(v3);
  let tot = omega3 + omega2 - omega1;

  // check angle is between
  if (tot < 0.001 && tot > -0.001) {
    v1 = a.sub(b);
    v2 = c.sub(b);
    v3 = point.clone().sub(b);

    omega1 = v1.angleTo(v2);
    omega2 = v2.angleTo(v3);
    omega3 = v1.angleTo(v3);
    tot = omega3 + omega2 - omega1;
    //debugger;

    if (tot < 0.001 && tot > -0.001) {
      return true;
    }
  }
  return false;
}

//////////////////
//              //
//   controls   //
//              //
//////////////////
const raycaster = new THREE.Raycaster();
const pointer = new THREE.Vector2();

const _plane = new THREE.Plane();
const _target = new Vector3();

const _cameraNormal = new Vector3();

let focusedEvent = "";

let intersects = [];

function onPointerMove(event) {
  // calculate pointer position in normalized device coordinates
  // (-1 to +1) for both components
  pointer.x = (event.clientX / window.innerWidth) * 2 - 1;
  pointer.y = -(event.clientY / window.innerHeight) * 2 + 1;
}

function onPointerClick(event, objects, scene) {
  if (focusedEvent && !withinBounds(_target))
    return console.log("outta bounds");
  switch (focusedEvent) {
    case "SPAWN_SPHERE":
      addObject(constructObject(_target, "SPHERE"), objects, scene);
      break;
    case "SPAWN_CUBE":
      addObject(constructObject(_target, "CUBE"), objects, scene);
      break;
    default:
      randomForceOnObject(intersects, objects);
      break;
  }

  focusedEvent = "";
}

function randomForceOnObject(intersects, objects) {
  for (let i = 0; i < intersects.length; i++) {
    const obj = getGameObjectFrom(intersects[i].object, objects);

    if (obj) {
      obj.applyForce(function () {
        return new ForceVector(
          new Vector3(
            (Math.random() - 0.5) * 40,
            250,
            (Math.random() - 0.5) * 40,
          ),
          obj.position,
        );
      });
    }
  }
}

function getGameObjectFrom(graphicalObject, objects) {
  return objects.filter((obj) => obj.graphicsObject === graphicalObject)[0];
}

const lowerBounds = {
  x: -7,
  y: 0,
  z: -7,
};
const upperBounds = {
  x: 7,
  y: 14,
  z: 7,
};

function withinBounds(vector3) {
  return (
    vector3.x > lowerBounds.x &&
    vector3.x < upperBounds.x &&
    vector3.y > lowerBounds.y &&
    vector3.y < upperBounds.y &&
    vector3.z > lowerBounds.z &&
    vector3.z < upperBounds.z
  );
}

/**
 *
 * @param {THREE.Scene} scene
 * @param {THREE.PerspectiveCamera} camera
 * @param {THREE.WebGLRenderer} renderer
 */
function updateScreen(scene, camera, renderer) {
  // update the picking ray with the camera and pointer position
  raycaster.setFromCamera(pointer, camera);
  intersects = raycaster.intersectObjects(
    objects.filter((obj) => obj.needsUpdate).map((obj) => obj.graphicsObject),
  );
  for (let i = 0; i < intersects.length; i++) {
    //intersects[i].object.material.color.set( 0x00ff00 );
  }

  camera.getWorldDirection(_cameraNormal);

  let pos = camera.position.length() - 8;
  pos = pos > 10 ? 5 : pos;
  _plane.set(_cameraNormal, pos);

  // calculate objects intersecting the picking ray
  raycaster.ray.intersectPlane(_plane, _target);

  renderer.render(scene, camera);
}

/**
 *
 * @param {THREE.PerspectiveCamera} camera
 * @param {THREE.OrbitControls} controls
 */
function updateCamera(camera) {
  camera.near = camera.position.clone().length() - 7;
  if (camera.near < 0.5) camera.near = 0.5;
  camera.updateProjectionMatrix();
}

window.addEventListener("pointermove", onPointerMove);

//////////////////
//              //
//   Features   //
//              //
//////////////////
let material = new THREE.MeshToonMaterial({ color: 0x4477aa });

let cubeShape = new THREE.BoxGeometry(1, 1, 1);
let sphereShape = new THREE.SphereGeometry(1, 10, 10);

function constructObject(position, type) {
  let object;
  switch (type) {
    case "SPHERE":
      object = new GameObject(new THREE.Mesh(sphereShape, material));
      break;
    case "CUBE":
      object = new GameObject(new THREE.Mesh(cubeShape, material));
      break;
    default:
      object = new GameObject(new THREE.Mesh(cubeShape, material));
      break;
  }

  object.velocity.x = 0.0;
  object.mass = 1;
  object.elasticity = 0.9;
  object.setPosition(position.x, position.y, position.z);

  return object;
}

/////////////////
//             //
//     Run     //
//             //
/////////////////

const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(
  75,
  window.innerWidth / window.innerHeight,
  3,
  1000,
);
const renderer = new THREE.WebGLRenderer();

const controls = new OrbitControls(camera, renderer.domElement);

controls.maxDistance = 50;

controls.addEventListener("change", () => updateCamera(camera));

const objects = [];
const worldForces = [];

camera.position.set(0, 2, 7);
camera.rotation.set(-0, 0, 0);

scene.background = new THREE.Color(0xdddddd);

let wallMaterial = new THREE.MeshToonMaterial({ color: 0x7777bb });

let plane = new THREE.BoxGeometry(15, 0.1, 15);
let walls = new THREE.BoxGeometry(0.5, 15, 15);
let floor = new THREE.Mesh(plane, wallMaterial);
let ceiling = new THREE.Mesh(plane, wallMaterial);
let walls1 = new THREE.Mesh(walls, wallMaterial);
let walls2 = new THREE.Mesh(walls, wallMaterial);
let walls3 = new THREE.Mesh(walls, wallMaterial);
let walls4 = new THREE.Mesh(walls, wallMaterial);

floor.position.x = 0;
ceiling.position.y = 15;
walls1.position.x = -7.5;
walls2.position.x = 7.5;
walls1.position.y = 7.5;
walls2.position.y = 7.5;
walls3.position.z = -7.5;
walls4.position.z = 7.5;
walls3.position.y = 7.5;
walls4.position.y = 7.5;

walls3.rotation.y = Math.PI / 2;
walls4.rotation.y = Math.PI / 2;

let floors = new GameObject(floor);
let floors2 = new GameObject(ceiling);
let wall1 = new GameObject(walls1);
let wall2 = new GameObject(walls2);
let wall3 = new GameObject(walls3);
let wall4 = new GameObject(walls4);

floors.needsUpdate = false;
floors.mass = 1;
floors2.needsUpdate = false;
wall1.needsUpdate = false;
wall2.needsUpdate = false;
wall3.needsUpdate = false;
wall4.needsUpdate = false;
wall2.mass = 1;

addObject(floors, objects, scene);
addObject(floors2, objects, scene);
addObject(wall1, objects, scene);
addObject(wall2, objects, scene);
addObject(wall3, objects, scene);
addObject(wall4, objects, scene);

const pointLight = new THREE.PointLight(0xffffff, 1, 100);
pointLight.position.set(10, 20, 10);
scene.add(pointLight);

const sphereSize = 1;
const pointLightHelper = new THREE.PointLightHelper(pointLight, sphereSize);
//scene.add( pointLightHelper );

const helper = new THREE.PlaneHelper(_plane, 10, 0xffff00);
//scene.add( helper );

worldForces.push((pos, mass) => {
  return new ForceVector(new Vector3(0, -9.8 * mass, 0), new Vector3(0, 0, 0));
});

const obj1 = constructObject({ x: 0, y: 5, z: 0 }, "SPHERE");

addObject(obj1, objects, scene);

function addListeners() {}

let renderDiv;

function main() {
  addListeners();

  document
    .getElementById("box-button")
    .addEventListener("click", () => (focusedEvent = "SPAWN_CUBE"));
  document
    .getElementById("ball-button")
    .addEventListener("click", () => (focusedEvent = "SPAWN_SPHERE"));

  renderDiv = document.getElementById("renderer");
  setInterval(() => {
    updateScreen(scene, camera, renderer);
    updatePhysics(objects, 1, { worldForces });
    controls.update();
  }, 10);
  renderDiv.wi;
  document.getElementById("renderer").appendChild(renderer.domElement);
  resize();
}

function resize() {
  renderer.setSize(renderDiv.offsetWidth, renderDiv.offsetHeight);
}

window.addEventListener("load", main);
window.addEventListener("resize", resize);

window.addEventListener("click", (e) => onPointerClick(e, objects, scene));
