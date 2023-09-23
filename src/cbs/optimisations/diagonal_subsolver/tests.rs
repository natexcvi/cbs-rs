use super::*;
use rstest::rstest;

#[rstest]
#[case(
    vec![
        Agent {
            start: (0, 2),
            goal: (9, 9),
            id: "0".to_string(),
        },
        Agent {
            start: (2, 0),
            goal: (9, 7),
            id: "1".to_string(),
        },
        Agent {
            start: (1, 1),
            goal: (9, 8),
            id: "2".to_string(),
        },
        Agent {
            start: (4, 4),
            goal: (7, 7),
            id: "3".to_string(),
        },
    ],
    Grid::new(10, 10, Grid::to_conditional_obstacles(vec![]),
    (0, 0)),
    vec![
        (
            Diagonal {
                direction: DiagonalDirection::Up,
                half: DiagonalHalf::Right,
                offset: 2,
            },
            vec![0, 1, 2],
        ),
        (
            Diagonal {
                direction: DiagonalDirection::Up,
                half: DiagonalHalf::Right,
                offset: 8,
            },
            vec![3],
        ),
    ].into_iter().collect(),
)]
fn test_find_diagonal_sets(
    #[case] agents: Vec<Agent>,
    #[case] scenario: Grid,
    #[case] expected: HashMap<Diagonal, Vec<usize>>,
) {
    let borrowed_agents = agents.iter().collect::<Vec<_>>();
    let mut exp_diag_sets = expected
        .into_iter()
        .map(|(diagonal, diag_agents)| {
            (
                diagonal,
                diag_agents
                    .iter()
                    .map(|agent_idx| &agents[*agent_idx])
                    .collect(),
            )
        })
        .collect::<HashMap<Diagonal, Vec<&Agent>>>();
    exp_diag_sets
        .values_mut()
        .for_each(|agents| agents.sort_by_key(|agent| agent.id.clone()));
    let mut diag_sets = find_diagonal_sets(borrowed_agents.iter(), &scenario);
    diag_sets
        .values_mut()
        .for_each(|agents| agents.sort_by_key(|agent| agent.id.clone()));
    assert_eq!(diag_sets, exp_diag_sets);
}

#[rstest]
#[case(
    vec![
        (
            Diagonal {
                direction: DiagonalDirection::Up,
                half: DiagonalHalf::Right,
                offset: 2,
            },
            vec![0, 1, 2],
        ),
        (
            Diagonal {
                direction: DiagonalDirection::Down,
                half: DiagonalHalf::Right,
                offset: 8,
            },
            vec![3],
        ),
        (
            Diagonal {
                direction: DiagonalDirection::Up,
                half: DiagonalHalf::Left,
                offset: 2,
            },
            vec![4],
        ),
        (
            Diagonal {
                direction: DiagonalDirection::Down,
                half: DiagonalHalf::Left,
                offset: 8,
            },
            vec![5],
        ),
    ].into_iter().collect(),
    (DiagonalDirection::Up, DiagonalHalf::Right),
)]
fn test_most_populous_diag_type(
    #[case] diagonals: HashMap<Diagonal, Vec<usize>>,
    #[case] expected: (DiagonalDirection, DiagonalHalf),
) {
    let agents = diagonals
        .values()
        .flatten()
        .map(|agent_idx| Agent {
            start: (0, 0),
            goal: (0, 0),
            id: agent_idx.to_string(),
        })
        .collect::<Vec<_>>();

    let (direction, half) = most_populous_diag_type(
        &diagonals
            .into_iter()
            .map(|(diag, diag_agents)| {
                (
                    diag,
                    diag_agents
                        .iter()
                        .map(|agent_idx| &agents[*agent_idx])
                        .collect::<Vec<_>>(),
                )
            })
            .collect(),
    );
    assert_eq!((direction, half), expected);
}

#[rstest]
#[case::sanity(
    Grid::new(10, 10, Grid::to_conditional_obstacles(vec![]),
    (0, 0)),
    vec![
        Agent {
            start: (0, 2),
            goal: (9, 9),
            id: "0".to_string(),
        },
    ],
    0,
    vec![
        vec![
            (0, 2),
            (1, 2),
            (2, 2),
            (3, 2),
            (4, 2),
            (5, 2),
            (6, 2),
            (7, 2),
            (8, 2),
            (9, 2),
            (9, 3),
            (9, 4),
            (9, 5),
            (9, 6),
            (9, 7),
            (9, 8),
            (9, 9),
        ],
    ],
)]
#[case::multiple_agents(
    Grid::new(10, 10, Grid::to_conditional_obstacles(vec![]),
    (0, 0)),
    vec![
        Agent {
            start: (0, 2),
            goal: (9, 9),
            id: "0".to_string(),
        },
        Agent {
            start: (0, 3),
            goal: (9, 8),
            id: "1".to_string(),
        },
        Agent {
            start: (1, 2),
            goal: (8, 7),
            id: "2".to_string(),
        },

    ],
    0,
    vec![
        vec![
            (0, 2),
            (1, 2),
            (2, 2),
            (3, 2),
            (4, 2),
            (5, 2),
            (6, 2),
            (7, 2),
            (7, 3),
            (7, 4),
            (7, 5),
            (7, 6),
            (7, 7),
            (7, 8),
            (8, 8),
            (8, 9),
            (9, 9),
        ],
        vec![
            (0, 3),
            (1, 3),
            (2, 3),
            (3, 3),
            (4, 3),
            (5, 3),
            (6, 3),
            (7, 3),
            (7, 4),
            (7, 5),
            (7, 6),
            (7, 7),
            (7, 8),
            (8, 8),
            (9, 8)
        ],
        vec![
            (1, 2),
            (2, 2),
            (3, 2),
            (4, 2),
            (5, 2),
            (6, 2),
            (7, 2),
            (8, 2),
            (8, 3),
            (8, 4),
            (8, 5),
            (8, 6),
            (8, 7),
        ],
    ],
)]
#[case::multiple_agents_no_individually_optimal_paths(
    Grid::new(10, 10, Grid::to_conditional_obstacles(vec![]),
    (0, 0)),
    vec![
        Agent {
            start: (0, 2),
            goal: (9, 9),
            id: "0".to_string(),
        },
        Agent {
            start: (0, 3),
            goal: (9, 8),
            id: "1".to_string(),
        },
        Agent {
            start: (1, 2),
            goal: (8, 7),
            id: "2".to_string(),
        },
        Agent {
            start: (1, 1),
            goal: (1, 9),
            id: "3".to_string(),
        },
        Agent {
            start: (0, 0),
            goal: (1, 0),
            id: "4".to_string(),
        },
    ],
    0,
    vec![
        vec![
            (0, 2),
            (0, 2),
            (1, 2),
            (2, 2),
            (3, 2),
            (4, 2),
            (5, 2),
            (6, 2),
            (7, 2),
            (7, 3),
            (7, 4),
            (7, 5),
            (7, 6),
            (7, 7),
            (7, 8),
            (8, 8),
            (8, 9),
            (9, 9),
        ],
        vec![
            (0, 3),
            (1, 3),
            (2, 3),
            (3, 3),
            (4, 3),
            (5, 3),
            (6, 3),
            (7, 3),
            (7, 4),
            (7, 5),
            (7, 6),
            (7, 7),
            (7, 8),
            (8, 8),
            (9, 8),
        ],
        vec![
            (1, 2),
            (2, 2),
            (3, 2),
            (4, 2),
            (5, 2),
            (6, 2),
            (7, 2),
            (8, 2),
            (8, 3),
            (8, 4),
            (8, 5),
            (8, 6),
            (8, 7),
        ],
        vec![
            (1, 1),
            (1, 2),
            (1, 3),
            (1, 4),
            (1, 5),
            (1, 6),
            (1, 7),
            (1, 8),
            (1, 9),
        ],
        vec![
            (0, 0),
            (1, 0),
        ],
    ],
)]
fn test_plan_two_direction_agents(
    #[case] grid: Grid,
    #[case] agents: Vec<Agent>,
    #[case] slackness: i32,
    #[case] expected_paths: Vec<Vec<(i32, i32)>>,
) {
    let borrowed_agents = agents.iter().collect::<Vec<_>>();
    let solver = crate::cbs::low_level::AStarLowLevelSolver::new();
    let mut node = ConflictTreeNode::new_without_init(
        borrowed_agents,
        Vec::<Box<crate::cbs::high_level::Constraint>>::new(),
        &grid,
        HashMap::<&Agent, Vec<(i32, i32)>>::new(),
        None,
        None,
        None,
        false,
        &solver,
        Rc::new(crate::cbs::high_level::heuristic::ZeroHeuristic::new()),
    );

    plan_two_direction_agents(&mut node, slackness);
    let paths = node.paths;
    let expected_paths = expected_paths
        .into_iter()
        .enumerate()
        .map(|(i, path)| (&agents[i], path))
        .filter(|(_, path)| !path.is_empty())
        .collect::<HashMap<_, _>>();

    assert_eq!(paths.len(), expected_paths.len());
    assert_eq!(
        paths
            .into_iter()
            .map(|(a, p)| (a, p.iter().map(|(t, l)| (*t, *l)).collect()))
            .collect::<HashMap<&Agent, Vec<(i32, i32)>>>(),
        expected_paths
    );
}
