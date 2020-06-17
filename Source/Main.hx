/*

Basic View example in Away3d

Demonstrates:

How to create a 3D environment for your objects
How to add a new textured object to your world
How to rotate an object in your world

Code by Rob Bateman
rob@infiniteturtles.co.uk
http://www.infiniteturtles.co.uk

This code is distributed under the MIT License

Copyright (c) The Away Foundation http://www.theawayfoundation.org

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the “Software”), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

*/

package;

import away3d.containers.*;
import away3d.entities.*;
import away3d.materials.*;
import away3d.primitives.*;
import away3d.utils.*;
import oimo.dynamics.rigidbody.RigidBodyType;

import openfl.display.*;
import openfl.events.*;
import openfl.geom.Vector3D;

import oimo.dynamics.World;
import oimo.common.Vec3;
import oimo.common.MathUtil;
import oimo.dynamics.rigidbody.RigidBody;


class Main extends Sprite {
	
	//engine variables
	private var view:View3D;
	
	//scene objects
	private var ground:Mesh;
	
	private var oimo_world:World;
	
	private var zeroVector3D:Vector3D = new Vector3D();
	
	/**
	 * Constructor
	 */
	public function new() {
		super();
		
		stage.scaleMode = StageScaleMode.NO_SCALE;
		stage.align = StageAlign.TOP_LEFT;
		
		var fps = new FPS();
		stage.addChild(fps);
		
		oimo_world = new World(null, new Vec3(0, -9.80665, 0));
		
		// first step, must do before anything else !!!
		OimoUtils.setWorld(oimo_world);
		
		// setup the view
		view = new View3D();
		addChild(view);
		
		// setup the camera
		view.camera.z = -2500;
		view.camera.y = 2000;
		view.camera.lookAt(zeroVector3D);
		view.camera.lens.far = 100000;
		view.backgroundColor = 0x999999;
		view.camera.transform.appendRotation(-70, Vector3D.Y_AXIS, zeroVector3D);
		
		var textureMat = new TextureMaterial(Cast.bitmapTexture("assets/floor_diffuse.jpg"));
		textureMat.repeat = true;
		textureMat.animateUVs = true;
		
		var grayMaterial = new TextureMaterial(Cast.bitmapTexture("assets/marble.jpg"));
		grayMaterial.repeat = true;
		grayMaterial.animateUVs = true;
		
		var orangeMaterial = new TextureMaterial(Cast.bitmapTexture("assets/orange.jpg"));
		orangeMaterial.repeat = true;
		orangeMaterial.animateUVs = true;
		
		// setup the scene
		ground = new Mesh(new CubeGeometry(1700, 10, 1700), grayMaterial);
		ground.subMeshes[0].scaleU = 8;
		ground.subMeshes[0].scaleV = 8;
		view.scene.addChild(ground);
		OimoUtils.addPhysics(ground, RigidBodyType.STATIC);	
		
		var staticSphere1:Mesh = new Mesh(new SphereGeometry(150), orangeMaterial);
		staticSphere1.subMeshes[0].scaleU = 4;
		staticSphere1.subMeshes[0].scaleV = 4;
		staticSphere1.moveTo(-400, 300, -400);
		view.scene.addChild(staticSphere1);	
		OimoUtils.addPhysics(staticSphere1, RigidBodyType.STATIC, [-400, 300, -400]);
		
		var staticSphere2:Mesh = new Mesh(new SphereGeometry(170), orangeMaterial);
		staticSphere2.subMeshes[0].scaleU = 4;
		staticSphere2.subMeshes[0].scaleV = 4;
		staticSphere2.moveTo(400, 300, -400);
		view.scene.addChild(staticSphere2);	
		OimoUtils.addPhysics(staticSphere2, RigidBodyType.STATIC, [400, 300, -400]);
		
		var staticSphere1:Mesh = new Mesh(new SphereGeometry(100), orangeMaterial);
		staticSphere1.subMeshes[0].scaleU = 4;
		staticSphere1.subMeshes[0].scaleV = 4;
		staticSphere1.moveTo(400, 300, 400);
		view.scene.addChild(staticSphere1);	
		OimoUtils.addPhysics(staticSphere1, RigidBodyType.STATIC, [400, 300, 400]);
		
		var staticSphere1:Mesh = new Mesh(new SphereGeometry(200), orangeMaterial);
		staticSphere1.subMeshes[0].scaleU = 4;
		staticSphere1.subMeshes[0].scaleV = 4;
		staticSphere1.moveTo(-400, 300, 400);
		view.scene.addChild(staticSphere1);	
		OimoUtils.addPhysics(staticSphere1, RigidBodyType.STATIC, [ -400, 300, 400]);
		
		var staticBox:Mesh = new Mesh(new CubeGeometry(200, 200, 200), orangeMaterial);
		staticBox.subMeshes[0].scaleU = 4;
		staticBox.subMeshes[0].scaleV = 4;
		staticBox.moveTo(0, 400, 0);
		staticBox.transform.appendRotation(Math.PI / 6 * OimoUtils.radToDeg, staticBox.rightVector, staticBox.position);	
		staticBox.transform.appendRotation(Math.PI / 5 * OimoUtils.radToDeg, staticBox.upVector, staticBox.position);					
		staticBox.transform.appendRotation(Math.PI / 3 * OimoUtils.radToDeg, staticBox.forwardVector, staticBox.position);
		view.scene.addChild(staticBox);	
		var sbPhys = OimoUtils.addPhysics(staticBox, RigidBodyType.STATIC, [0, 400, 0]);
		sbPhys.setRotationXyz(new Vec3(Math.PI / 6, Math.PI / 5, Math.PI / 3));
		
		var cubeGeom = new CubeGeometry(100, 100, 100);
		for (i in 0...50) {
			var box = new Mesh(cubeGeom, textureMat);
			view.scene.addChild(box);		
			OimoUtils.addPhysics(box, RigidBodyType.DYNAMIC, [OimoUtils.getRandomFloat(-500, 500), 300 + (i * 201), OimoUtils.getRandomFloat(-500, 500)]);
		}
		
		var capsuleGeom = new CapsuleGeometry(50, 100, 16, 11);
		for (i in 0...50) {
			var sphere = new Mesh(capsuleGeom, textureMat);
			view.scene.addChild(sphere);		
			OimoUtils.addPhysics(sphere, RigidBodyType.DYNAMIC, [OimoUtils.getRandomFloat(-500, 500), 3000 + (i * 201), OimoUtils.getRandomFloat(-500, 500)]);
		}
		
		var sphereGeom = new SphereGeometry(50);
		for (i in 0...50) {
			var sphere = new Mesh(sphereGeom, textureMat);
			view.scene.addChild(sphere);		
			OimoUtils.addPhysics(sphere, RigidBodyType.DYNAMIC, [OimoUtils.getRandomFloat(-500, 500), 5000 + (i * 201), OimoUtils.getRandomFloat(-500, 500)]);
		}
		
		var cylinderGeom = new CylinderGeometry(50, 50, 120);
		for (i in 0...50) {
			var sphere = new Mesh(cylinderGeom, textureMat);
			view.scene.addChild(sphere);		
			OimoUtils.addPhysics(sphere, RigidBodyType.DYNAMIC, [OimoUtils.getRandomFloat(-500, 500), 7000 + (i * 201), OimoUtils.getRandomFloat(-500, 500)]);
		}
		
		// setup the render loop
		addEventListener(Event.ENTER_FRAME, _onEnterFrame);
		stage.addEventListener(Event.RESIZE, onResize);
		onResize();
	}
	
	/**
	 * render loop
	 */
	private function _onEnterFrame(e:Event):Void {
		OimoUtils.updatePhysics();
		
		teleportRigidBodies(-800, 1500, 500, 500);
		
		view.render();
	}
	
	var tmpVec3_0:Vec3 = new Vec3();
	var tmpVec3_1:Vec3 = new Vec3();
	function teleportRigidBodies(thresholdY:Float, toY:Float, rangeX:Float, rangeZ:Float):Void {
		var rb:RigidBody = oimo_world.getRigidBodyList();
		tmpVec3_0.zero();
		while (rb != null) {
			rb.getPositionTo(tmpVec3_1);
			if (tmpVec3_1.y < thresholdY) {
				tmpVec3_1.y = toY;
				tmpVec3_1.x = MathUtil.randIn(-1, 1) * rangeX;
				tmpVec3_1.z = MathUtil.randIn(-1, 1) * rangeZ;
				rb.setPosition(tmpVec3_1);
				rb.setLinearVelocity(tmpVec3_0);
			}
			rb = rb.getNext();
		}
	}
	
	/**
	 * stage listener for resize events
	 */
	private function onResize(event:Event = null):Void {
		view.width = stage.stageWidth;
		view.height = stage.stageHeight;
	}
	
}
