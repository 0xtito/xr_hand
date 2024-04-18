mod constants;

use bevy::transform::TransformSystem;
use bevy_rapier3d::prelude::*;
use bevy_rapier3d::plugin::{RapierConfiguration, TimestepMode};


use constants::*;

use std::time::Duration;
use bevy::{ecs::schedule::ScheduleLabel, prelude::*};


// We can create our own gizmo config group!
#[derive(Default, Reflect, GizmoConfigGroup)]
struct MyRoundGizmos {}

fn main() {

    let mut app = App::new();

    app
    .add_plugins(DefaultPlugins)
    .add_plugins(RapierPhysicsPlugin::<NoUserData>::default().with_default_system_setup(false))
    // .add_plugins(RapierDebugRenderPlugin::default())
    .init_gizmo_group::<MyRoundGizmos>()
    .add_systems(Startup, setup)
    // .add_systems(Startup, spawn_hand_entities) 
    .add_systems(Startup, (spawn_hand_entities.before(spawn_physics_hands), spawn_physics_hands))
    // .add_systems(Startup, (spawn_physics_hands))
    .add_systems(
        FixedUpdate,
        update_physics_hands.before(PhysicsSet::SyncBackend),
    );

    app.configure_sets(
        PostUpdate,
        (
            PhysicsSet::SyncBackend,
            PhysicsSet::StepSimulation,
            PhysicsSet::Writeback,
        )
            .chain()
            .before(TransformSystem::TransformPropagate),
    );

    //configure rapier sets
    let mut physics_schedule = Schedule::new(PhysicsSchedule);

    physics_schedule.configure_sets(
        (
            PhysicsSet::SyncBackend,
            PhysicsSet::StepSimulation,
            PhysicsSet::Writeback,
        )
            .chain()
            .before(TransformSystem::TransformPropagate),
    );

    //add rapier systems
    physics_schedule.add_systems((
        RapierPhysicsPlugin::<NoUserData>::get_systems(PhysicsSet::SyncBackend)
            .in_set(PhysicsSet::SyncBackend),
        RapierPhysicsPlugin::<NoUserData>::get_systems(PhysicsSet::StepSimulation)
            .in_set(PhysicsSet::StepSimulation),
        RapierPhysicsPlugin::<NoUserData>::get_systems(PhysicsSet::Writeback)
            .in_set(PhysicsSet::Writeback),
    ));
    app.add_schedule(physics_schedule) // configure our fixed timestep schedule to run at the rate we want
        .insert_resource(Time::<Fixed>::from_duration(Duration::from_secs_f32(
            FIXED_TIMESTEP,
        )))
        .add_systems(FixedUpdate, run_physics_schedule)
        .add_systems(Startup, configure_physics);

    app.run()
}


fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {

    commands.spawn(Camera3dBundle {
        camera: Camera {
            ..default()
        },
        transform: Transform::from_xyz(0.0, 1.0, 0.25).with_rotation(
            Quat::from_xyzw(-0.16057923, -0.5977889, 0.76988935, -0.15534931),
        ) 
            .looking_at(Vec3::new(0.11578956, 1.0322298, -0.07940306), Vec3::Y),
        ..default()
    });

    // light
    commands.spawn(PointLightBundle {
        point_light: PointLight {
            shadows_enabled: true,
            ..default()
        },
        transform: Transform::from_xyz(4.0, 8.0, 4.0),
        ..default()
    });
}

// A label for our new Schedule!
#[derive(ScheduleLabel, Debug, Hash, PartialEq, Eq, Clone)]
struct PhysicsSchedule;

fn run_physics_schedule(world: &mut World) {
    world.run_schedule(PhysicsSchedule);
}

fn configure_physics(mut rapier_config: ResMut<RapierConfiguration>) {
    rapier_config.timestep_mode = TimestepMode::Fixed {
        dt: FIXED_TIMESTEP,
        substeps: 1,
    }
}

