extern crate nalgebra as na;

use na::{ Point2, Vector2 };
use nphysics2d::world::World;
use nphysics_testbed2d::Testbed;
use ncollide2d::shape::{ ShapeHandle, Ball };
use nphysics2d::object::{ Body, BodyPart, RigidBody, ColliderDesc, RigidBodyDesc};
use nphysics2d::force_generator::Spring;

fn main() {
    let mut world = World::<f32>::new();

    let col = ColliderDesc::new(ShapeHandle::new(Ball::new(0.2)))
        .density(1.0);

    let mut rb_desc1 = RigidBodyDesc::new()
        .collider(&col);

    rb_desc1.set_translation(Vector2::new(0.0, 2.0));
    let rbph1 = rb_desc1.build(&mut world).part(0).unwrap().part_handle();
    rb_desc1.set_translation(Vector2::new(0.0, -2.0));
    let rbph2 = rb_desc1.build(&mut world).part(0).unwrap().part_handle();

    rb_desc1.set_translation(Vector2::new(1.0, -2.0));
    let rbph3 = rb_desc1.build(&mut world).part(0).unwrap().part_handle();

    rb_desc1.set_translation(Vector2::new(-1.0, 3.0));
    let rbph4 = rb_desc1.build(&mut world).part(0).unwrap().part_handle();

    rb_desc1.set_translation(Vector2::new(0.5, -2.0));
    let rbph5 = rb_desc1.build(&mut world).part(0).unwrap().part_handle();





    world.add_force_generator(Spring::new(rbph1, rbph2, Point2::new(0.0,0.0), Point2::new(0.0,0.0), 3.0, 5.0));
    world.add_force_generator(Spring::new(rbph1, rbph3, Point2::new(0.0,0.0), Point2::new(0.0,0.0), 3.0, 5.0));
    world.add_force_generator(Spring::new(rbph1, rbph4, Point2::new(0.0,0.0), Point2::new(0.0,0.0), 3.0, 5.0));
    world.add_force_generator(Spring::new(rbph4, rbph2, Point2::new(0.0,0.0), Point2::new(0.0,0.0), 3.0, 5.0));
    world.add_force_generator(Spring::new(rbph1, rbph5, Point2::new(0.0,0.0), Point2::new(0.0,0.0), 3.0, 5.0));



    let mut testbed = Testbed::new(world);
    testbed.look_at(Point2::new(0.0, -3.0), 100.0);
    testbed.run();
}
