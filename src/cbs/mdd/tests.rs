use crate::cbs::low_level::LocationTime;

use super::*;
use rstest::rstest;
#[rstest]
#[case::simple(
crate::cbs::low_level::Grid::new(10, 10, Grid::to_conditional_obstacles(vec![]), (0, 0)),
(0, 0),
(5, 5),
10,
Ok(vec![
    vec![(0,0)],
    vec![(1,0),(0,1)],
    vec![(2,0),(1,1),(0,2)],
    vec![(3,0),(2,1),(1,2),(0,3)],
    vec![(4,0),(3,1),(2,2),(1,3),(0,4)],
    vec![(5,0),(4,1),(3,2),(2,3),(1,4),(0,5)],
    vec![(5,1), (4,2), (3,3), (2,4), (1,5)],
    vec![(5,2), (4,3), (3,4), (2,5)],
    vec![(5,3), (4,4), (3,5)],
    vec![(5,4), (4,5)],
    vec![(5,5)],
])
)]
#[case::with_obstacles(
crate::cbs::low_level::Grid::new(10, 10, Grid::to_conditional_obstacles(vec![
    LocationTime::new((1, 1), -1),
    LocationTime::new((1, 2), -1),
    LocationTime::new((1, 3), -1),
]), (0, 0)),
(0, 0),
(5, 5),
10,
Ok(vec![
    vec![(0,0)],
    vec![(1,0),(0,1)],
    vec![(2,0),(0,2)],
    vec![(3,0),(2,1),(0,3)],
    vec![(4,0),(3,1),(2,2),(0,4)],
    vec![(5,0),(4,1),(3,2),(2,3),(1,4),(0,5)],
    vec![(5,1), (4,2), (3,3), (2,4), (1,5)],
    vec![(5,2), (4,3), (3,4), (2,5)],
    vec![(5,3), (4,4), (3,5)],
    vec![(5,4), (4,5)],
    vec![(5,5)],
])
)]
#[case::goal_unreachable_error(
crate::cbs::low_level::Grid::new(10, 10, Grid::to_conditional_obstacles(vec![]), (0, 0)),
(0, 0),
(5, 6),
10,
Err(MDDError::GoalUnreachable),
)]
fn test_mdd(
    #[case] scenario: crate::cbs::low_level::Grid,
    #[case] start: (i32, i32),
    #[case] goal: (i32, i32),
    #[case] c: i32,
    #[case] expected: Result<Vec<Vec<(i32, i32)>>, MDDError>,
) {
    let agent = crate::cbs::high_level::Agent {
        id: "a".to_string(),
        start,
        goal,
    };
    let mdd = super::mdd(&agent, &scenario, c);
    match mdd {
        Ok(mdd) => {
            assert_eq!(mdd, expected.unwrap());
        }
        Err(_) => {
            assert!(expected.is_err());
        }
    }
}
