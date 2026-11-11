// A robot on the Clunkyard core's Rust binding. The core owns tracking,
// geometry and protocol handling; this strategy chooses the tactics.
// Clunkyard robot API 2.0.0: one core under every language, the same
// answers everywhere. Facts (a call shows its arguments):
// match: mode ruleset control robots is_3d tick_rate max_ticks
//   robot_id(slot)
// clock: tick seconds remaining_ticks
// arena: size gravity terrain_kind ground_height(x y)
// me: health position velocity heading pitch roll angular_velocity
//   attitude grounded altitude applied
// battery: charge capacity regen sustainable_drive compute
//   drive_cost(fraction)
// drive: hover_lift
// gun: aim_now cooldown_ticks spread max_power damage_scale
//   can_fire(power reserve) shot_speed(power) shot_cost(power)
//   reload_ticks(power) shot_damage(power) power_for_speed(speed)
//   power_for_flight_time(range seconds) lead(contact power)
//   lead_point(position velocity power)
// scanner: last proximity_range reach(arc) arc_for_reach(range)
// sensors: contacts nearest cues contact(slot) position_of(contact)
// tracker: cue nearest_bearing_rate velocity(slot) position(slot)
//   last_seen(slot)
// nav: zone zone.margin(point) zone.contains(point margin)
//   zone.distance_from(point) storm storm.margin(point)
//   storm.contains(point margin) storm.distance_from(point) hill
//   hill.margin(point) hill.contains(point margin)
//   hill.distance_from(point) pickups nearest_pickup(kind)
//   field_distance(a b)
// score: me best armour
// angles: normalize(degrees) angle_diff(target current)
//   bearing_to(from to) elevation_to(from to) direction(heading pitch)
//   distance(a b) clamp(value low high)
// raw: number(path) text(path) has(path) flag(path) count(path) line()
// Controls: drive.set(throttle turn pitch_turn)
//   drive.forces(thrust strafe lift yaw_torque pitch_torque roll_torque brake)
//   gun.aim(heading pitch) gun.fire(power) scanner.set(heading arc pitch)
//   tracker.configure(smoothing forget_ticks cue_ticks)
//   status.set(text value) raw.extra(json)
// Every control argument is optional; a second call keeps what the first
// set. Absent facts are your language's none. Names are snake_case here;
// your language cases them its own way. Angles are degrees from the +x
// axis; pitch is positive up.
// Manual: https://clunkyard.com/manual#sdk
use clunkyard::{Error, Robot, Strategy};

#[derive(Default)]
struct Starter { free_flight: bool }
fn steer(robot: &Robot, towards: f64, current: f64) -> Result<f64, Error> {
    robot.angles.clamp(robot.angles.angle_diff(towards, current)? / 45.0, -1.0, 1.0)
}
// The arena's calibration kernel, xorshift-sqrt v1; safe to delete.
fn calibration_work(work: u64, tick: u64) -> f64 {
    let mut x = 88172645463325252u64 ^ tick;
    let mut sum = 0.0;
    for _ in 0..work { x ^= x << 13; x ^= x >> 7; x ^= x << 17; sum += ((x >> 11) as f64).sqrt(); }
    ((sum.floor() as u64 + (x >> 40)) % 1000000) as f64
}
impl Strategy for Starter {
    fn on_start(&mut self, robot: &Robot) -> Result<(), Error> {
        self.free_flight = robot.r#match.is_3d()? && robot.arena.gravity()?.z == 0.0;
        Ok(())
    }
    fn on_turn(&mut self, robot: &Robot) -> Result<(), Error> {
        let position = robot.me.position()?;
        let (heading, pitch, tick) = (robot.me.heading()?, robot.me.pitch()?, robot.clock.tick()?);
        robot.drive.forces(None, None, None, None, None, None, Some(0.2))?;
        if let Some(target) = robot.sensors.nearest()? {
            let throttle = if target.range > 350.0 { 0.7 } else if target.range < 200.0 { -0.4 } else { 0.2 };
            robot.drive.set(Some(throttle), Some(steer(robot, target.bearing, heading)?), Some(steer(robot, target.elevation, pitch)?))?;
            robot.scanner.set(Some(target.bearing), Some(20.0), Some(target.elevation))?;
            let power = robot.gun.power_for_flight_time(target.range, 1.0)?;
            let aim = robot.gun.lead(&target, power)?;
            let (towards, up) = aim.map(|aim| (aim.heading, aim.pitch)).unwrap_or((target.bearing, target.elevation));
            robot.gun.aim(Some(towards), Some(up))?;
            let now = robot.gun.aim_now()?;
            if robot.angles.angle_diff(towards, now.heading)?.abs() < 3.0 && (up - now.pitch).abs() < 3.0 && robot.gun.can_fire(power, 10.0)? { robot.gun.fire(Some(power))?; }
            robot.status.set(Some("TRACK"), Some(target.range))?;
        } else if let Some(cue) = robot.tracker.cue()? {
            let bearing = robot.angles.bearing_to(&position, &cue.position)?;
            let distance = robot.angles.distance(&position, &cue.position)?;
            let elevation = if self.free_flight { robot.angles.elevation_to(&position, &cue.position)? } else { 0.0 };
            let error = cue.bearing_error.min(30.0);
            let across = (tick % 5 - 2) as f64 / 2.0 * error;
            let up = if self.free_flight { ((tick / 5) % 3 - 1) as f64 * error / 2.0 } else { 0.0 };
            robot.drive.set(Some(0.7), Some(steer(robot, bearing, heading)?), Some(steer(robot, elevation, pitch)?))?;
            robot.scanner.set(Some(bearing + across), Some(robot.scanner.arc_for_reach(distance + 100.0)?), Some(elevation + up))?;
            robot.status.set(Some("CUE"), Some(distance))?;
        } else {
            robot.drive.set(Some(0.5), Some(0.33), Some(steer(robot, 0.0, pitch)?))?;
            let up = if self.free_flight { ((tick / 10) % 3 - 1) as f64 * 35.0 } else { 0.0 };
            robot.scanner.set(Some(robot.angles.normalize(robot.scanner.last()?.heading + 36.0)?), Some(40.0), Some(up))?;
            robot.status.set(Some("SEARCH"), None)?;
        }
        if robot.battery.charge()? < robot.battery.capacity()? / 4.0 {
            robot.drive.set(Some(robot.drive.get_throttle()?.unwrap_or(0.0) * 0.5), None, None)?;
        }
        if let Some(storm) = robot.nav.storm()? {
            if storm.margin(&position)?.unwrap_or(f64::NEG_INFINITY) > -60.0 {
                let away = robot.angles.bearing_to(&storm.center, &position)?;
                robot.drive.set(Some(0.8), Some(robot.angles.clamp(robot.angles.angle_diff(away, heading)? / 30.0, -1.0, 1.0)?), None)?;
            }
        }
        if let Some(zone) = robot.nav.zone()? {
            if zone.margin(&position)?.unwrap_or(f64::INFINITY) < 60.0 {
                let bearing = robot.angles.bearing_to(&position, &zone.center)?;
                let climb = if zone.shape == clunkyard::SHAPE_SPHERE { robot.angles.elevation_to(&position, &zone.center)? } else { 0.0 };
                robot.drive.set(Some(0.8), Some(robot.angles.clamp(robot.angles.angle_diff(bearing, heading)? / 30.0, -1.0, 1.0)?), Some(robot.angles.clamp((climb - pitch) / 30.0, -1.0, 1.0)?))?;
            }
        }
        if let Some(work) = robot.raw.number("state.calibration.work")? {
            if work >= 1.0 { robot.status.set(Some("WORK"), Some(calibration_work(work as u64, tick as u64)))?; }
        }
        Ok(())
    }
}
fn main() -> Result<(), Error> { clunkyard::run(Starter::default()) }
