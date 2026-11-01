use ecs_workload::Position;

use crate::Orientation3d;

/// Corner order shared with the browser renderers: x varies fastest, then y, then z.
const CORNER_SIGNS: [[f64; 3]; 8] = [
    [-1.0, -1.0, -1.0],
    [1.0, -1.0, -1.0],
    [-1.0, 1.0, -1.0],
    [1.0, 1.0, -1.0],
    [-1.0, -1.0, 1.0],
    [1.0, -1.0, 1.0],
    [-1.0, 1.0, 1.0],
    [1.0, 1.0, 1.0],
];

/// The eight integer corners of an oriented box, rounded to the nearest spatial unit.
#[must_use]
pub fn oriented_box_vertices(
    center: Position,
    half_extents: [i32; 3],
    orientation: Orientation3d,
) -> [Position; 8] {
    let [qx, qy, qz, qw] = orientation.to_unit();
    let length = (qx * qx + qy * qy + qz * qz + qw * qw).sqrt();
    let [x, y, z, w] = [qx, qy, qz, qw].map(|value| value / length);
    let matrix = [
        [
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - z * w),
            2.0 * (x * z + y * w),
        ],
        [
            2.0 * (x * y + z * w),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - x * w),
        ],
        [
            2.0 * (x * z - y * w),
            2.0 * (y * z + x * w),
            1.0 - 2.0 * (x * x + y * y),
        ],
    ];
    let half = half_extents.map(f64::from);
    CORNER_SIGNS.map(|signs| {
        let local = [signs[0] * half[0], signs[1] * half[1], signs[2] * half[2]];
        let offset = matrix.map(|row| row[0] * local[0] + row[1] * local[1] + row[2] * local[2]);
        Position::new3(
            center.x + offset[0].round() as i64,
            center.y + offset[1].round() as i64,
            center.z + offset[2].round() as i64,
        )
    })
}

#[cfg(test)]
mod tests {
    use ecs_workload::Position;

    use super::oriented_box_vertices;
    use crate::Orientation3d;

    #[test]
    fn identity_corners_follow_the_renderer_order() {
        let corners = oriented_box_vertices(
            Position::new3(10, 20, 30),
            [1, 2, 3],
            Orientation3d::IDENTITY,
        );
        assert_eq!(corners[0], Position::new3(9, 18, 27));
        assert_eq!(corners[1], Position::new3(11, 18, 27));
        assert_eq!(corners[2], Position::new3(9, 22, 27));
        assert_eq!(corners[7], Position::new3(11, 22, 33));
    }

    #[test]
    fn quarter_turn_about_z_swaps_x_and_y_extents() {
        let quarter = Orientation3d::from_axis_angle([0.0, 0.0, 1.0], std::f64::consts::FRAC_PI_2);
        let corners = oriented_box_vertices(Position::new3(0, 0, 0), [100, 10, 1], quarter);
        let max_x = corners.iter().map(|corner| corner.x).max();
        let max_y = corners.iter().map(|corner| corner.y).max();
        assert_eq!(max_x, Some(10));
        assert_eq!(max_y, Some(100));
    }
}
