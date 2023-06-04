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
    Grid::new(10, 10, Grid::to_conditional_obstacles(vec![]), (0, 0)),
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
    assert_eq!(diag_sets, exp_diag_sets,);
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
