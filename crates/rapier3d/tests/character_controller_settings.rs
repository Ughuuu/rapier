//! The `KinematicCharacterController` settings that used to be hard-coded: the slide and
//! depenetration budgets, the ground prediction distance and normal threshold, and the
//! kinematic-platform switch.

use rapier3d::control::{
    CharacterLength, EffectiveCharacterMovement, KinematicCharacterController,
};
use rapier3d::prelude::*;

const RADIUS: Real = 0.5;

fn world_with_ground() -> PhysicsWorld {
    let mut world = PhysicsWorld::new();
    world.insert_collider(
        ColliderBuilder::cuboid(100.0, 0.1, 100.0).translation(Vector::new(0.0, -0.1, 0.0)),
        None,
    );
    world.step();
    world
}

fn move_ball(
    world: &PhysicsWorld,
    controller: &KinematicCharacterController,
    pos: Vector,
    desired_translation: Vector,
) -> EffectiveCharacterMovement {
    controller.move_shape(
        world.integration_parameters.dt,
        &world.query_pipeline(),
        &Ball::new(RADIUS),
        &Pose::from_translation(pos),
        desired_translation,
        |_| {},
    )
}

#[test]
fn controller_defaults_match_the_former_constants() {
    let controller = KinematicCharacterController::default();
    assert_eq!(controller.max_slide_iterations, 20);
    assert_eq!(
        controller.max_depenetration_distance,
        CharacterLength::Relative(0.25)
    );
    assert_eq!(controller.max_depenetration_iterations, 4);
    assert_eq!(
        controller.ground_prediction_distance,
        CharacterLength::Absolute(0.05)
    );
    assert_eq!(controller.min_ground_normal_dot, 1.0e-3);
    assert!(controller.follow_kinematic_platforms);
}

#[test]
fn zero_slide_iterations_keep_the_character_in_place() {
    let world = world_with_ground();
    let pos = Vector::new(0.0, RADIUS + 0.01, 0.0);
    let desired = Vector::new(0.1, 0.0, 0.0);

    let moved = move_ball(
        &world,
        &KinematicCharacterController::default(),
        pos,
        desired,
    );
    assert!(moved.translation.x > 0.09);

    let controller = KinematicCharacterController {
        max_slide_iterations: 0,
        snap_to_ground: None,
        ..Default::default()
    };
    let stuck = move_ball(&world, &controller, pos, desired);
    assert_eq!(stuck.translation, Vector::ZERO);
}

#[test]
fn the_depenetration_budget_caps_the_push_out_of_an_overlap() {
    let mut world = PhysicsWorld::new();
    // A box whose top face sits 0.3 inside the ball.
    world.insert_collider(
        ColliderBuilder::cuboid(5.0, 0.5, 5.0).translation(Vector::new(0.0, -0.5, 0.0)),
        None,
    );
    world.step();
    let pos = Vector::new(0.0, RADIUS - 0.3, 0.0);

    let default_push = move_ball(
        &world,
        &KinematicCharacterController::default(),
        pos,
        Vector::ZERO,
    )
    .translation
    .y;
    // The default budget is a quarter of the ball’s height: 0.25.
    assert!((default_push - 0.25).abs() < 1.0e-5, "{default_push}");

    let capped = KinematicCharacterController {
        max_depenetration_distance: CharacterLength::Absolute(0.1),
        ..Default::default()
    };
    let capped_push = move_ball(&world, &capped, pos, Vector::ZERO).translation.y;
    assert!((capped_push - 0.1).abs() < 1.0e-5, "{capped_push}");

    let disabled = KinematicCharacterController {
        max_depenetration_iterations: 0,
        ..Default::default()
    };
    let no_push = move_ball(&world, &disabled, pos, Vector::ZERO).translation;
    assert_eq!(no_push, Vector::ZERO);
}

#[test]
fn a_longer_ground_prediction_grounds_a_character_further_up() {
    let world = world_with_ground();
    // Hovering 0.2 above the ground: past the default offset + 0.05.
    let pos = Vector::new(0.0, RADIUS + 0.2, 0.0);
    let controller = KinematicCharacterController {
        snap_to_ground: None,
        ..Default::default()
    };
    assert!(!move_ball(&world, &controller, pos, Vector::ZERO).grounded);

    let controller = KinematicCharacterController {
        ground_prediction_distance: CharacterLength::Absolute(0.3),
        ..controller
    };
    assert!(move_ball(&world, &controller, pos, Vector::ZERO).grounded);
}

#[test]
fn a_ground_normal_threshold_above_one_never_grounds() {
    let world = world_with_ground();
    let pos = Vector::new(0.0, RADIUS + 0.01, 0.0);
    let controller = KinematicCharacterController::default();
    assert!(move_ball(&world, &controller, pos, Vector::ZERO).grounded);

    let controller = KinematicCharacterController {
        min_ground_normal_dot: 1.1,
        ..controller
    };
    assert!(!move_ball(&world, &controller, pos, Vector::ZERO).grounded);
}

/// Stands a character on a platform sliding along +x for one second, returning how far the
/// character moved along x.
fn distance_carried_by_platform(controller: &KinematicCharacterController) -> Real {
    let mut world = world_with_ground();
    world.insert(
        RigidBodyBuilder::kinematic_velocity_based()
            .translation(Vector::new(0.0, 0.3, 0.0))
            .linvel(Vector::new(1.0, 0.0, 0.0)),
        ColliderBuilder::cuboid(5.0, 0.1, 5.0),
    );
    let start = Vector::new(0.0, 0.4 + RADIUS + 0.01, 0.0);
    let mut pos = start;
    for _ in 0..60 {
        world.step();
        // A little gravity, so the controller runs its shape-casting loop.
        pos += move_ball(&world, controller, pos, Vector::new(0.0, -0.01, 0.0)).translation;
    }
    pos.x - start.x
}

#[test]
fn a_character_follows_a_kinematic_platform_unless_disabled() {
    let carried = distance_carried_by_platform(&KinematicCharacterController::default());
    assert!(carried > 0.9, "carried {carried}");

    let left_behind = distance_carried_by_platform(&KinematicCharacterController {
        follow_kinematic_platforms: false,
        ..Default::default()
    });
    assert!(left_behind.abs() < 1.0e-5, "carried {left_behind}");
}
