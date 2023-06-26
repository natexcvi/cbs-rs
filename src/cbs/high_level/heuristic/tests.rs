use crate::cbs::{
    low_level::{AStarLowLevelSolver, Grid, LocationTime},
};

use super::*;
use rstest::rstest;

#[rstest]
#[case(
    Grid::new(
        5, 
        5,
        Grid::to_conditional_obstacles(vec![]),
        (0, 0),
    ),
    vec![
        vec![
            (0, 0),
            (0, 1),
            (0, 2),
            (0, 3),
            (0, 4),
        ],
        vec![
            (4, 0),
            (4, 1),
            (4, 2),
            (4, 3),
            (4, 4),
        ],
    ],
    vec![],
    0.0
)]
#[case::cardinal_edge_conflict(
    Grid::new(
        2, 
        3,
        Grid::to_conditional_obstacles(vec![]),
        (0, 0),
    ),
    vec![
        vec![
            (0, 0),
            (0, 1),
            (0, 2),
            (1, 2),
        ],
        vec![
            (1, 0),
            (1, 1),
            (1, 2),
            (0, 2),
        ],
    ],
    vec![],
    1.0
)]
#[case::cardinal_rectangle_conflict(
    Grid::new(
        3, 
        3,
        Grid::to_conditional_obstacles(vec![]),
        (0, 0),
    ),
    vec![
        vec![
            (1, 0),
            (1, 1),
            (1, 2),
        ],
        vec![
            (0, 1),
            (1, 1),
            (2, 1),
        ],
    ],
    vec![],
    1.0
)]
#[case::cardinal_vertex_conflict(
    Grid::new(
        3, 
        3,
        Grid::to_conditional_obstacles(vec![
            LocationTime::new((0, 1), -1),
            LocationTime::new((2, 1), -1),
        ]),
        (0, 0),
    ),
    vec![
        vec![
            (0, 0),
            (1, 0),
            (1, 1),
            (1, 2),
            (0, 2),
        ],
        vec![
            (2, 0),
            (1, 0),
            (1, 1),
            (1, 2),
            (2, 2),
        ],
    ],
    vec![],
    1.0
)]
fn test_dg_heuristic(
    #[case] scenario: Grid,
    #[case] paths: Vec<Path>,
    #[case] constraints: Vec<Box<crate::cbs::high_level::Constraint>>,
    #[case] expected_h: f64,
) {
    let agents: Vec<Agent> = paths
        .iter()
        .enumerate()
        .map(|(i, path)| Agent {
            id: i.to_string(),
            start: path.first().unwrap().clone(),
            goal: path.last().unwrap().clone(),
        })
        .collect();
    let precomputed_paths = agents
        .iter()
        .map(|agent| (agent, paths[agent.id.parse::<usize>().unwrap()].clone()))
        .collect();
    let heuristic: Rc<dyn Heuristic> = Rc::new(DGHeuristic::new());
    let solver = AStarLowLevelSolver::new();
    let node = ConflictTreeNode::new(
        agents.iter().collect(),
        constraints,
        precomputed_paths,
        &scenario,
        None,
        None,
        None,
        true,
        &solver,
        Rc::clone(&heuristic),
    );
    let h = heuristic.h(&node);
    assert_eq!(h, expected_h);
}
