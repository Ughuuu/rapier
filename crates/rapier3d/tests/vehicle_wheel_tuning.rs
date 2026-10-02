//! The per-wheel tuning knobs of `DynamicRayCastVehicleController`: `roll_influence`, the
//! skid factors, and the `skid_info` they produce.

use rapier3d::control::{DynamicRayCastVehicleController, WheelTuning};
use rapier3d::prelude::*;

struct Vehicle {
    world: PhysicsWorld,
    controller: DynamicRayCastVehicleController,
}

impl Vehicle {
    fn new(tuning: WheelTuning) -> Self {
        let mut world = PhysicsWorld::new();
        let ground =
            ColliderBuilder::cuboid(200.0, 0.1, 200.0).translation(Vector::new(0.0, -0.1, 0.0));
        world.insert_collider(ground, None);

        let hw = 0.3;
        let hh = 0.15;
        let chassis = RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 0.3, 0.0));
        let collider = ColliderBuilder::cuboid(hw * 2.0, hh, hw).density(100.0);
        let (handle, _) = world.insert(chassis, collider);

        let mut controller = DynamicRayCastVehicleController::new(handle);
        for pos in [
            Vector::new(hw * 1.5, -hh, hw),
            Vector::new(hw * 1.5, -hh, -hw),
            Vector::new(-hw * 1.5, -hh, hw),
            Vector::new(-hw * 1.5, -hh, -hw),
        ] {
            controller.add_wheel(pos, -Vector::Y, Vector::Z, hh, hh / 4.0, &tuning);
        }

        let mut vehicle = Self { world, controller };
        // Let the suspension settle.
        for _ in 0..50 {
            vehicle.step();
        }
        vehicle
    }

    fn chassis(&mut self) -> &mut RigidBody {
        &mut self.world.bodies[self.controller.chassis]
    }

    fn step(&mut self) {
        let chassis = self.controller.chassis;
        let queries = self.world.broad_phase.as_query_pipeline_mut(
            self.world.narrow_phase.query_dispatcher(),
            &mut self.world.bodies,
            &mut self.world.colliders,
            QueryFilter::exclude_dynamic().exclude_rigid_body(chassis),
        );
        self.controller
            .update_vehicle(self.world.integration_parameters.dt, queries);
        self.world.step();
    }
}

fn tuning() -> WheelTuning {
    WheelTuning {
        suspension_stiffness: 100.0,
        suspension_damping: 10.0,
        friction_slip: 2.0,
        ..WheelTuning::default()
    }
}

/// Launches the vehicle and brakes hard, returning the smallest `skid_info` seen on any wheel
/// while braking.
fn min_skid_info_while_braking(tuning: WheelTuning) -> Real {
    let mut vehicle = Vehicle::new(tuning);
    let linvel = vehicle.chassis().linvel();
    vehicle
        .chassis()
        .set_linvel(Vector::new(10.0, linvel.y, 0.0), true);

    let mut min_skid_info = Real::MAX;
    for _ in 0..30 {
        for wheel in vehicle.controller.wheels_mut() {
            wheel.brake = 1.0e9;
        }
        vehicle.step();
        for wheel in vehicle.controller.wheels() {
            min_skid_info = min_skid_info.min(wheel.skid_info());
        }
    }
    min_skid_info
}

#[test]
fn wheel_tuning_defaults_match_the_former_constants() {
    let tuning = WheelTuning::default();
    assert_eq!(tuning.roll_influence, 0.1);
    assert_eq!(tuning.forward_skid_factor, 0.5);
    assert_eq!(tuning.side_skid_factor, 1.0);

    let mut controller = DynamicRayCastVehicleController::new(RigidBodyHandle::invalid());
    let wheel = controller.add_wheel(
        Vector::ZERO,
        -Vector::Y,
        Vector::Z,
        0.15,
        0.05,
        &WheelTuning {
            roll_influence: 0.3,
            forward_skid_factor: 0.2,
            side_skid_factor: 0.7,
            ..WheelTuning::default()
        },
    );
    assert_eq!(wheel.roll_influence, 0.3);
    assert_eq!(wheel.forward_skid_factor, 0.2);
    assert_eq!(wheel.side_skid_factor, 0.7);
    assert_eq!(wheel.skid_info(), 0.0);
}

#[test]
fn a_rolling_wheel_grips_and_a_hard_braking_wheel_skids() {
    let vehicle = Vehicle::new(tuning());
    for wheel in vehicle.controller.wheels() {
        assert_eq!(wheel.skid_info(), 1.0);
    }

    let min_skid_info = min_skid_info_while_braking(tuning());
    assert!(
        min_skid_info < 0.5,
        "braking far past the grip limit must skid, got skid_info {min_skid_info}"
    );
}

#[test]
fn a_zero_forward_skid_factor_never_skids_on_braking_alone() {
    let min_skid_info = min_skid_info_while_braking(WheelTuning {
        // No side impulse, so only the forward impulse could exceed the grip.
        side_friction_stiffness: 0.0,
        forward_skid_factor: 0.0,
        ..tuning()
    });
    assert_eq!(min_skid_info, 1.0);
}

/// Turns at speed and returns the largest angle between the chassis’ up axis and the world’s.
fn max_roll_in_a_turn(roll_influence: Real) -> Real {
    let mut vehicle = Vehicle::new(WheelTuning {
        roll_influence,
        friction_slip: 10.5,
        ..tuning()
    });
    let mut max_roll: Real = 0.0;
    for i in 0..240 {
        for (k, wheel) in vehicle.controller.wheels_mut().iter_mut().enumerate() {
            wheel.engine_force = 30.0;
            wheel.steering = if k < 2 && i > 60 { 0.5 } else { 0.0 };
        }
        vehicle.step();
        let up = *vehicle.chassis().rotation() * Vector::Y;
        max_roll = max_roll.max(up.angle_between(Vector::Y));
    }
    max_roll
}

#[test]
fn a_larger_roll_influence_rolls_the_chassis_more_in_a_turn() {
    let without_roll = max_roll_in_a_turn(0.0);
    let with_roll = max_roll_in_a_turn(1.0);
    assert!(
        with_roll > without_roll * 1.5,
        "roll_influence 1.0 rolled {with_roll} rad, 0.0 rolled {without_roll} rad"
    );
}
