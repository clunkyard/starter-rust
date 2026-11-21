// A robot on the Clunkyard core's Rust binding. The core owns tracking,
// geometry and protocol handling; this strategy chooses the tactics.
// It searches with a broad scan, narrows the beam on a contact, and leads
// shots from a line through its recent sightings.
// Clunkyard robot API 2.1.0: one core under every language, the same
// answers everywhere. Facts (a call shows its arguments):
// match: mode ruleset control robots is_3d tick_rate max_ticks
//   robot_id(slot)
// clock: tick seconds remaining_ticks
// arena: size gravity terrain_kind ground_height(x y)
// me: health position velocity heading pitch roll angular_velocity
//   attitude grounded altitude wall_contact hurt applied hull_mounts
//   to_mount(heading pitch) from_mount(heading pitch) mount_toward(point)
// battery: charge capacity regen sustainable_drive compute
//   drive_cost(fraction)
// drive: hover_lift
// gun: aim_now cooldown_ticks spread max_power damage_scale
//   can_fire(power reserve) shot_speed(power) shot_cost(power)
//   reload_ticks(power) shot_damage(power) power_for_speed(speed)
//   power_for_flight_time(range seconds) lead(contact power)
//   lead_point(position velocity power)
// scanner: last proximity_range reach(arc) arc_for_reach(range) is_on
// sensors: contacts nearest cues contact(slot) position_of(contact)
// tracker: cue nearest_bearing_rate velocity(slot) position(slot)
//   history(slot) last_seen(slot)
// nav: zone zone.margin(point) zone.contains(point margin)
//   zone.distance_from(point) storm storm.margin(point)
//   storm.contains(point margin) storm.distance_from(point) hill
//   hill.margin(point) hill.contains(point margin)
//   hill.distance_from(point) wall_margin() pickups nearest_pickup(kind)
//   field_distance(a b)
// score: me best armour
// drone: has ready in_flight launches_left report sightings
// angles: normalize(degrees) angle_diff(target current)
//   bearing_to(from to) elevation_to(from to) direction(heading pitch)
//   distance(a b) to_chassis(heading pitch attitude)
//   from_chassis(heading pitch attitude) clamp(value low high)
// raw: number(path) text(path) has(path) flag(path) count(path) line()
// Controls: drive.set(throttle turn pitch_turn)
//   drive.forces(thrust strafe lift yaw_torque pitch_torque roll_torque brake)
//   gun.aim(heading pitch) gun.fire(power) scanner.set(heading arc pitch)
//   scanner.power(on) tracker.configure(smoothing forget_ticks cue_ticks)
//   drone.launch(heading pitch param) drone.brief(index value)
//   drone.program(language source) status.set(text value) debug.log(line)
//   raw.extra(json)
// Every control argument is optional; a second call keeps what the first
// set. Absent facts are your language's none. Names are snake_case here;
// your language cases them its own way. Angles are degrees from the +x
// axis; pitch is positive up.
// Manual: https://clunkyard.com/manual#sdk
use clunkyard::{Error, Robot, Strategy, Vec3};

// Bearing is earned: a scan contact's bearing is off by up to its bearing_error
// (degrees, a fixed share of the arc), so a wide beam finds a robot and a narrow
// one aims at it. Contacts within 120 units are exact.
const SEARCH_ARC: f64 = 40.0; // degrees: sweeps the circle in half a second
const HOLD_WIDTH: f64 = 60.0; // units across at the target: it stays in the beam from tick to tick
const MIN_ARC: f64 = 6.0;     // degrees: the narrowest the beam holds
const WINDOW: i32 = 12;       // ticks of sightings the aim averages
const SIGHTINGS: usize = 8;   // narrow sightings in the window before a shot
const LOOK_EVERY: i32 = 40;   // ticks between looks around the target, for a nearer robot
const REACQUIRE: i32 = 10;    // ticks to look again where a lost target was

#[derive(Default)]
struct Starter { free_flight: bool, dodge_ticks: u32, dodges: u32, slot: i32 }
// A straight line through the narrowest of the last WINDOW ticks' sightings:
// where the robot is now and how fast it moves. Each sighting is off by its
// bearing_error; the line averages that out. Returns the position, velocity and sightings used.
fn fit(robot: &Robot, slot: i32, tick: i32) -> Result<Option<(Vec3, Vec3, usize)>, Error> {
    let recent: Vec<_> = robot.tracker.history(slot)?.into_iter().filter(|s| s.tick > tick - WINDOW).collect();
    if recent.len() < 2 { return Ok(None); }
    let best = recent.iter().map(|s| s.bearing_error).fold(f64::INFINITY, f64::min);
    let seen: Vec<_> = recent.iter().filter(|s| s.bearing_error <= 1.5 * best).collect();
    if seen.len() < 2 { return Ok(None); }
    let n = seen.len() as f64;
    let t0 = seen.iter().map(|s| s.tick as f64).sum::<f64>() / n;
    let spread: f64 = seen.iter().map(|s| (s.tick as f64 - t0).powi(2)).sum();
    let rate = robot.r#match.tick_rate()? as f64;
    // Least squares on one axis: its fitted position now and its speed per second.
    let line = |axis: fn(&Vec3) -> f64| {
        let mean = seen.iter().map(|s| axis(&s.position)).sum::<f64>() / n;
        let slope = seen.iter().map(|s| (s.tick as f64 - t0) * (axis(&s.position) - mean)).sum::<f64>() / spread;
        (mean + slope * (tick as f64 - t0), slope * rate)
    };
    let (x, y, z) = (line(|p| p.x), line(|p| p.y), line(|p| p.z));
    Ok(Some((Vec3 { x: x.0, y: y.0, z: z.0 }, Vec3 { x: x.1, y: y.1, z: z.1 }, seen.len())))
}
fn steer(robot: &Robot, towards: f64, current: f64) -> Result<f64, Error> {
    robot.angles.clamp(robot.angles.angle_diff(towards, current)? / 45.0, -1.0, 1.0)
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
        let last_seen = if self.slot != 0 { robot.tracker.last_seen(self.slot)? } else { None };
        if let Some(target) = robot.sensors.nearest()? {
            self.slot = target.slot;
            let throttle = if target.range > 350.0 { 0.7 } else if target.range < 200.0 { -0.4 } else { 0.2 };
            robot.drive.set(Some(throttle), Some(steer(robot, target.bearing, heading)?), Some(steer(robot, target.elevation, pitch)?))?;
            // Halve the beam each tick until it is HOLD_WIDTH across at the target:
            // narrow enough for a precise bearing, wide enough to keep it in.
            let hold = MIN_ARC.max(2.0 * (HOLD_WIDTH / 2.0).atan2(target.range).to_degrees());
            // In a crowd a narrow beam sees one robot. Every LOOK_EVERY ticks, widen it
            // for a tick: a nearer robot it finds becomes the target.
            let arc = if tick % LOOK_EVERY == 0 { SEARCH_ARC } else { hold.max(SEARCH_ARC.min(robot.scanner.last()?.arc / 2.0)) };
            robot.scanner.set(Some(target.bearing), Some(arc), Some(target.elevation))?;
            let power = robot.gun.power_for_flight_time(target.range, 1.0)?;
            // An exact contact: gun.lead is the intercept if the target holds course.
            // A noisy one: lead the fitted line, and fire only once it rests on enough sightings.
            let (aim, ready) = if target.bearing_error > 0.0 {
                match fit(robot, target.slot, tick)? {
                    Some((position, velocity, count)) => { let aim = robot.gun.lead_point(&position, &velocity, power)?; let ready = aim.is_some() && count >= SIGHTINGS; (aim, ready) }
                    None => (None, false),
                }
            } else { (robot.gun.lead(&target, power)?, true) };
            let (towards, up) = aim.map(|aim| (aim.heading, aim.pitch)).unwrap_or((target.bearing, target.elevation));
            robot.gun.aim(Some(towards), Some(up))?;
            let now = robot.gun.aim_now()?;
            if ready && robot.angles.angle_diff(towards, now.heading)?.abs() < 3.0 && (up - now.pitch).abs() < 3.0 && robot.gun.can_fire(power, 10.0)? { robot.gun.fire(Some(power))?; }
            robot.status.set(Some(if ready { "TRACK" } else { "NARROW" }), Some(target.range))?;
        } else if last_seen.is_some_and(|seen| tick - seen <= REACQUIRE) {
            // Lost it: look again where it was last seen, widening the beam, before searching.
            let at = robot.tracker.position(self.slot)?.unwrap_or(position);
            let bearing = robot.angles.bearing_to(&position, &at)?;
            let elevation = if self.free_flight { robot.angles.elevation_to(&position, &at)? } else { 0.0 };
            robot.drive.set(Some(0.5), Some(steer(robot, bearing, heading)?), Some(steer(robot, elevation, pitch)?))?;
            robot.scanner.set(Some(bearing), Some(SEARCH_ARC.min(robot.scanner.last()?.arc * 2.0)), Some(elevation))?;
            robot.status.set(Some("REACQUIRE"), None)?;
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
            robot.scanner.set(Some(robot.angles.normalize(robot.scanner.last()?.heading + 36.0)?), Some(SEARCH_ARC), Some(up))?;
            robot.status.set(Some("SEARCH"), None)?;
        }
        // A simple dodge: vary it, or predict the shooter.
        if self.dodge_ticks == 0 && robot.sensors.cues()?.iter().any(|heard| heard.source == clunkyard::SOURCE_MUZZLE) {
            self.dodges += 1;
            self.dodge_ticks = 10;
        }
        if self.dodge_ticks > 0 {
            // Odd dodges go left, even ones right; turn +1 and strafe -1 go left.
            let left = self.dodges % 2 == 1;
            self.dodge_ticks -= 1;
            if robot.r#match.control()? == clunkyard::CONTROL_FORCES {
                robot.drive.set(Some(0.5), None, None)?;
                robot.drive.forces(None, Some(if left { -1.0 } else { 1.0 }), None, None, None, None, None)?;
            } else {
                robot.drive.set(Some(1.0), Some(if left { 1.0 } else { -1.0 }), None)?;
            }
            robot.status.set(Some(if left { "dodge:left" } else { "dodge:right" }), Some(self.dodges as f64))?;
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
        Ok(())
    }
}
fn main() -> Result<(), Error> { clunkyard::run(Starter::default()) }
