use rstest::rstest;

use super::*;
use proptest::prelude::*;

#[rstest]
#[case::simple(
    Grid::new(
        5,
        5,
        Grid::to_conditional_obstacles(
            vec![
                LocationTime {
                    location: (1, 0),
                    time: -1,
                },
                LocationTime {
                    location: (1, 1),
                    time: -1,
                },
                LocationTime {
                    location: (1, 2),
                    time: -1,
                },
                LocationTime {
                    location: (1, 3),
                    time: -1,
                },
            ]
            .into_iter()
            .collect(),
        ),
        (2, 0),
    ),
    (0, 0),
    10.0,
)]
#[case::non_start_node(
    Grid::new(
        5,
        5,
        Grid::to_conditional_obstacles(
            vec![
                LocationTime {
                    location: (1, 0),
                    time: -1,
                },
                LocationTime {
                    location: (1, 1),
                    time: -1,
                },
                LocationTime {
                    location: (1, 2),
                    time: -1,
                },
                LocationTime {
                    location: (1, 3),
                    time: -1,
                },
                LocationTime {
                    location: (3, 0),
                    time: -1,
                },
            ]
            .into_iter()
            .collect(),
        ),
        (2, 0),
    ),
    (4, 0),
    4.0,
)]
fn test_true_distance(#[case] grid: Grid, #[case] query: (i32, i32), #[case] true_distance: f64) {
    let td = TrueDistance::new(Rc::new(grid), (0, 0));
    let h = td.h(&LocationTime {
        location: query,
        time: 0,
    });
    assert_eq!(h, true_distance);
}

prop_compose! {
    fn arb_location()(x in 0..5000, y in 0..5000) -> Location {
        (x, y)
    }
}

proptest! {
    #[rstest]
    fn test_dynamic_goal_manhattan_distance(
        first_goal in arb_location(),
        second_goal in arb_location(),
        query in arb_location(),
        time in -1..5000
    ) {
        let td = DynamicGoalManhattanDistance::new(first_goal);
        let h = td.h(&LocationTime {
            location: query,
            time,
        });
        assert_eq!(
            h,
            (first_goal.0 - query.0).abs() as f64 + (first_goal.1 - query.1).abs() as f64
        );
        td.set_goal(second_goal);
        let h = td.h(&LocationTime {
            location: query,
            time,
        });
        assert_eq!(
            h,
            (second_goal.0 - query.0).abs() as f64 + (second_goal.1 - query.1).abs() as f64
        );
    }
}
