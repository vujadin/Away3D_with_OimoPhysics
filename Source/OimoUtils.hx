package;

import away3d.core.math.Quaternion;
import away3d.entities.Mesh;
import away3d.primitives.CubeGeometry as AwayCubeGeom;
import away3d.primitives.SphereGeometry as AwaySphereGeom;
import away3d.primitives.PlaneGeometry as AwayPlaneGeom;
import away3d.primitives.CapsuleGeometry as AwayCapsuleGeom;
import away3d.primitives.ConeGeometry as AwayConeGeom;
import away3d.primitives.CylinderGeometry as AwayCylinderGeom;
import away3d.primitives.TorusGeometry as AwayTorusGeom;
import openfl.Lib;
import openfl.geom.Orientation3D;
import openfl.geom.Vector3D;
import openfl.utils.Timer;

import oimo.collision.geometry.*;
import oimo.common.*;
import oimo.dynamics.*;
import oimo.dynamics.constraint.joint.*;
import oimo.dynamics.rigidbody.*;


class OimoUtils {
	
	private static var oimoWorld:World;
	
	private static var tmpVec3_0:Vec3 = new Vec3();
	private static var tmpVec3_1:Vec3 = new Vec3();
	
	private static var oimoStaticBodies:Array<RigidBody> = [];
	private static var oimoDynamicBodies:Array<RigidBody> = [];
	private static var oimoKinematicBodies:Array<RigidBody> = [];
	
	private static var awayStaticBodies:Array<Mesh> = [];
	private static var awayDynamicBodies:Array<Mesh> = [];
	private static var awayKinematicBodies:Array<Mesh> = [];
	
	private static var lastTime:Float = 0;
	
	public static inline var radToDeg:Float = 57.295;

	
	public static function setWorld(w:World):Void {
		if (oimoWorld == null) {
			oimoWorld = w;
		}
		else {
			throw "Error: World already set.";
		}
	}
	
	public static function updatePhysics():Void {		
		oimoWorld.step(120 / 1000);
		
		for (i in 0...oimoDynamicBodies.length) {
			var oimoBody = oimoDynamicBodies[i];
			var awayBody = awayDynamicBodies[i];
			
			tmpVec3_0 = oimoBody.getTransform().getRotation().toEulerXyz();			
			awayBody.moveTo(oimoBody.getPosition().x, oimoBody.getPosition().y, oimoBody.getPosition().z);			
			awayBody.transform.appendRotation(tmpVec3_0.x * radToDeg, awayBody.rightVector, awayBody.position);	
			awayBody.transform.appendRotation(tmpVec3_0.y * radToDeg, awayBody.upVector, awayBody.position);					
			awayBody.transform.appendRotation(tmpVec3_0.z * radToDeg, awayBody.forwardVector, awayBody.position);
		}
	}
	
	public static inline function addPhysics(mesh:Mesh, type:Int = 0, pos:Array<Float> = null, colliderType:String = "", options:Dynamic = null):RigidBody {
		if (pos == null) {
			pos = [0, 0, 0];
		}
		tmpVec3_1.x = pos[0];
		tmpVec3_1.y = pos[1];
		tmpVec3_1.z = pos[2];
		
		var rBody:RigidBody = null;
		
		if (colliderType == "") {
			if (Std.is(mesh.geometry, AwayCubeGeom)) {
				var geom:AwayCubeGeom = cast mesh.geometry;
				tmpVec3_0.x = geom.width / 2;
				tmpVec3_0.y = geom.height / 2;
				tmpVec3_0.z = geom.depth / 2;
				rBody = OimoUtils.addBox(OimoUtils.oimoWorld, tmpVec3_1, tmpVec3_0, type);
			}
			else if (Std.is(mesh.geometry, AwaySphereGeom)) {
				var geom:AwaySphereGeom = cast mesh.geometry;
				var radius = geom.radius;
				rBody = OimoUtils.addSphere(OimoUtils.oimoWorld, tmpVec3_1, radius, type);
			}
			else if (Std.is(mesh.geometry, AwayCylinderGeom)) {
				var geom:AwayCylinderGeom = cast mesh.geometry;
				var radius = geom.topRadius;
				var halfHeight = geom.height / 2;
				rBody = OimoUtils.addCylinder(OimoUtils.oimoWorld, tmpVec3_1, radius, halfHeight, type);
			}
			else if (Std.is(mesh.geometry, AwayCapsuleGeom)) {
				var geom:AwayCapsuleGeom = cast mesh.geometry;
				var radius = geom.radius;
				var halfHeight = geom.height / 2;
				rBody = OimoUtils.addCapsule(OimoUtils.oimoWorld, tmpVec3_1, radius, halfHeight, type);
			}
		}
		else {
			
		}
		
		switch (type) {
			case RigidBodyType.STATIC:
				OimoUtils.oimoStaticBodies.push(rBody);
				OimoUtils.awayStaticBodies.push(mesh);
				
			case RigidBodyType.DYNAMIC:
				OimoUtils.oimoDynamicBodies.push(rBody);
				OimoUtils.awayDynamicBodies.push(mesh);
				
			case RigidBodyType.KINEMATIC:
				OimoUtils.oimoKinematicBodies.push(rBody);
				OimoUtils.awayKinematicBodies.push(mesh);
		}
		
		return rBody;
	}
	
	public static inline function getRandomInt(min:Int, max:Int):Int {
		return Math.floor(Math.random() * ((max + 1) - min)) + min;
	}
	
	public static function getRandomFloat(min:Float, max:Float):Float {
		return Math.random() * (max - min) + min;
	}

	public static function addUniversalJoint(w:World, rb1:RigidBody, rb2:RigidBody, anchor:Vec3, axis1:Vec3, axis2:Vec3, sd1:SpringDamper = null, lm1:RotationalLimitMotor = null, sd2:SpringDamper = null, lm2:RotationalLimitMotor = null):UniversalJoint {
		var jc:UniversalJointConfig = new UniversalJointConfig();
		jc.init(rb1, rb2, anchor, axis1, axis2);
		if (sd1 != null) jc.springDamper1 = sd1;
		if (lm1 != null) jc.limitMotor1 = lm1;
		if (sd2 != null) jc.springDamper2 = sd2;
		if (lm2 != null) jc.limitMotor2 = lm2;
		var j:UniversalJoint = new UniversalJoint(jc);
		w.addJoint(j);
		return j;
	}

	public static function addGenericJoint(w:World, rb1:RigidBody, rb2:RigidBody, anchor:Vec3, basis1:Mat3, basis2:Mat3, translSds:Array<SpringDamper> = null, translLms:Array<TranslationalLimitMotor> = null, rotSds:Array<SpringDamper> = null, rotLms:Array<RotationalLimitMotor> = null):GenericJoint {
		var jc:GenericJointConfig = new GenericJointConfig();
		jc.init(rb1, rb2, anchor, basis1, basis2);
		for (i in 0...3) {
			if (translSds != null && translSds[i] != null) jc.translationalSpringDampers[i] = translSds[i];
			if (translLms != null && translLms[i] != null) jc.translationalLimitMotors[i]   = translLms[i];
			if (rotSds != null    && rotSds[i] != null)    jc.rotationalSpringDampers[i]    = rotSds[i];
			if (rotLms != null    && rotLms[i] != null)    jc.rotationalLimitMotors[i]      = rotLms[i];
		}
		var j:GenericJoint = new GenericJoint(jc);
		w.addJoint(j);
		return j;
	}

	public static function addPrismaticJoint(w:World, rb1:RigidBody, rb2:RigidBody, anchor:Vec3, axis:Vec3, sd:SpringDamper = null, lm:TranslationalLimitMotor = null):PrismaticJoint {
		var jc:PrismaticJointConfig = new PrismaticJointConfig();
		jc.init(rb1, rb2, anchor, axis);
		if (sd != null) jc.springDamper = sd;
		if (lm != null) jc.limitMotor = lm;
		var j:PrismaticJoint = new PrismaticJoint(jc);
		w.addJoint(j);
		return j;
	}

	public static function addRevoluteJoint(w:World, rb1:RigidBody, rb2:RigidBody, anchor:Vec3, axis:Vec3, sd:SpringDamper = null, lm:RotationalLimitMotor = null):RevoluteJoint {
		var jc:RevoluteJointConfig = new RevoluteJointConfig();
		jc.init(rb1, rb2, anchor, axis);
		if (sd != null) jc.springDamper = sd;
		if (lm != null) jc.limitMotor = lm;
		var j:RevoluteJoint = new RevoluteJoint(jc);
		w.addJoint(j);
		return j;
	}

	public static function addCylindricalJoint(w:World, rb1:RigidBody, rb2:RigidBody, anchor:Vec3, axis:Vec3, rotSd:SpringDamper = null, rotLm:RotationalLimitMotor = null, traSd:SpringDamper = null, traLm:TranslationalLimitMotor = null):CylindricalJoint {
		var jc:CylindricalJointConfig = new CylindricalJointConfig();
		jc.init(rb1, rb2, anchor, axis);
		if (rotSd != null) jc.rotationalSpringDamper = rotSd;
		if (rotLm != null) jc.rotationalLimitMotor = rotLm;
		if (traSd != null) jc.translationalSpringDamper = traSd;
		if (traLm != null) jc.translationalLimitMotor = traLm;
		var j:CylindricalJoint = new CylindricalJoint(jc);
		w.addJoint(j);
		return j;
	}

	public static function addSphericalJoint(w:World, rb1:RigidBody, rb2:RigidBody, anchor:Vec3):SphericalJoint {
		var jc:SphericalJointConfig = new SphericalJointConfig();
		jc.init(rb1, rb2, anchor);
		var j:SphericalJoint = new SphericalJoint(jc);
		w.addJoint(j);
		return j;
	}

	public static function addSphere(w:World, center:Vec3, radius:Float, type:Int):RigidBody {
		return addRigidBody(w, center, new SphereGeometry(radius), type);
	}

	public static function addBox(w:World, center:Vec3, halfExtents:Vec3, type:Int):RigidBody {
		return addRigidBody(w, center, new BoxGeometry(halfExtents), type);
	}

	public static function addCylinder(w:World, center:Vec3, radius:Float, halfHeight:Float, type:Int):RigidBody {
		return addRigidBody(w, center, new CylinderGeometry(radius, halfHeight), type);
	}

	public static function addCone(w:World, center:Vec3, radius:Float, halfHeight:Float, type:Int):RigidBody {
		return addRigidBody(w, center, new ConeGeometry(radius, halfHeight), type);
	}

	public static function addCapsule(w:World, center:Vec3, radius:Float, halfHeight:Float, type:Int):RigidBody {
		return addRigidBody(w, center, new CapsuleGeometry(radius, halfHeight), type);
	}

	public static function addRigidBody(w:World, center:Vec3, geom:Geometry, type:Int):RigidBody {
		var shapec:ShapeConfig = new ShapeConfig();
		shapec.geometry = geom;
		var bodyc:RigidBodyConfig = new RigidBodyConfig();
		bodyc.type = type;
		bodyc.position = center;
		var body:RigidBody = new RigidBody(bodyc);
		body.addShape(new Shape(shapec));
		w.addRigidBody(body);
		return body;
	}

}
