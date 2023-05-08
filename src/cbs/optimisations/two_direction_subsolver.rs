use std::collections::{HashMap, HashSet};

use crate::cbs::high_level::{ConflictTreeNode, Path};
use crate::cbs::low_level::{Grid, LocationTime};
use crate::cbs::search::dfs;
use crate::cbs::Agent;

#[derive(Debug, PartialEq, Eq, Hash, Clone)]
enum DiagonalDirection {
    Up,
    Down,
}

#[derive(Debug, PartialEq, Eq, Hash, Clone)]
enum DiagonalHalf {
    Left,
    Right,
}

#[derive(Debug, PartialEq, Eq, Hash)]
struct Diagonal {
    direction: DiagonalDirection,
    half: DiagonalHalf,
    offset: i32,
}

impl Diagonal {
    fn direction_vecs(&self) -> Vec<(i32, i32)> {
        match self.direction {
            DiagonalDirection::Up => match self.half {
                DiagonalHalf::Left => vec![(-1, 0), (0, -1)],
                DiagonalHalf::Right => vec![(1, 0), (0, 1)],
            },
            DiagonalDirection::Down => match self.half {
                DiagonalHalf::Left => vec![(-1, 0), (0, 1)],
                DiagonalHalf::Right => vec![(1, 0), (0, -1)],
            },
        }
    }
}

pub fn plan_two_direction_agents(node: &mut ConflictTreeNode) {
    let diagonals = find_diagonal_sets(node.agents.iter(), &node.scenario);
    let (direction, half) = most_populous_diag_type(&diagonals);

    let mut chosen_diagonals = diagonals
        .into_iter()
        .filter(|(diagonal, _)| diagonal.direction == direction && diagonal.half == half)
        .collect::<Vec<(Diagonal, Vec<&Agent>)>>();

    chosen_diagonals.sort_by_key(
        |(diagonal, _)| match (&diagonal.direction, &diagonal.half) {
            (DiagonalDirection::Up, DiagonalHalf::Left) => diagonal.offset,
            (DiagonalDirection::Up, DiagonalHalf::Right) => -diagonal.offset,
            (DiagonalDirection::Down, DiagonalHalf::Left) => -diagonal.offset,
            (DiagonalDirection::Down, DiagonalHalf::Right) => diagonal.offset,
        },
    );

    let mut aux_grid = Grid::new(
        node.scenario.width,
        node.scenario.height,
        node.scenario.obstacles.clone().into_iter().collect(),
        node.scenario.goal,
    );
    for (diagonal, agents) in chosen_diagonals.iter() {
        let mut paths = HashMap::<&Agent, Path>::new();
        let mut target_obstacles = Vec::<LocationTime>::new();
        for agent in agents {
            let mut visited = HashSet::<LocationTime>::new();
            let mut path: Path = vec![agent.start];
            dfs(
                &mut visited,
                &mut path,
                &|path, cur, _| {
                    path.push(cur.location.clone());
                    true
                },
                &|path, _, _| {
                    path.pop();
                },
                LocationTime {
                    location: agent.start.clone(),
                    time: 0,
                },
                None,
                &|cur| {
                    diagonal
                        .direction_vecs()
                        .iter()
                        .map(|(dx, dy)| LocationTime {
                            location: (cur.location.0 + dx, cur.location.1 + dy),
                            time: cur.time + 1,
                        })
                        .filter(|loc| {
                            aux_grid.is_valid_location(&loc.location)
                                && is_in_start_goal_box(loc, agent)
                        })
                        .collect()
                },
                &|cur| cur.location == agent.goal,
            );
            paths.insert(agent, path);
            target_obstacles.push(LocationTime {
                location: agent.goal.clone(),
                time: -1,
            });
        }
        node.paths.extend(paths);
        aux_grid.obstacles.extend(target_obstacles);
    }
}

fn is_in_start_goal_box(loc_time: &LocationTime, agent: &Agent) -> bool {
    let (x, y) = loc_time.location;
    let (start_x, start_y) = agent.start;
    let (goal_x, goal_y) = agent.goal;
    (x >= start_x.min(goal_x) && x <= start_x.max(goal_x))
        && (y >= start_y.min(goal_y) && y <= start_y.max(goal_y))
}

fn most_populous_diag_type(
    diagonals: &HashMap<Diagonal, Vec<&Agent>>,
) -> (DiagonalDirection, DiagonalHalf) {
    let diagonal_types = vec![
        (DiagonalDirection::Up, DiagonalHalf::Left),
        (DiagonalDirection::Up, DiagonalHalf::Right),
        (DiagonalDirection::Down, DiagonalHalf::Left),
        (DiagonalDirection::Down, DiagonalHalf::Right),
    ];
    let (direction, half) = diagonal_types
        .iter()
        .map(|(direction, half)| {
            diagonals
                .iter()
                .filter(|(diagonal, _)| diagonal.direction == *direction && diagonal.half == *half)
                .map(|(_, agents)| agents.len())
                .sum::<usize>()
        })
        .enumerate()
        .max_by_key(|(_, count)| *count)
        .map(|(index, _)| diagonal_types[index].clone())
        .unwrap();
    (direction, half)
}

fn find_diagonal_sets<'a, 'b, I>(agents: I, scenario: &'a Grid) -> HashMap<Diagonal, Vec<&Agent>>
where
    I: Iterator<Item = &'b &'a Agent>,
    'a: 'b,
{
    let mut diagonals = HashMap::<Diagonal, Vec<&Agent>>::new();
    for agent in agents {
        let goal_down_right = agent.goal.0 >= agent.start.0 && agent.goal.1 >= agent.start.1;
        let goal_up_left = agent.goal.0 <= agent.start.0 && agent.goal.1 <= agent.start.1;
        let goal_down_left = agent.goal.0 <= agent.start.0 && agent.goal.1 >= agent.start.1;
        let diagonal = if goal_up_left {
            Diagonal {
                direction: DiagonalDirection::Up,
                half: DiagonalHalf::Left,
                offset: agent.start.0 + agent.start.1,
            }
        } else if goal_down_right {
            Diagonal {
                direction: DiagonalDirection::Up,
                half: DiagonalHalf::Right,
                offset: agent.start.0 + agent.start.1,
            }
        } else if goal_down_left {
            Diagonal {
                direction: DiagonalDirection::Down,
                half: DiagonalHalf::Left,
                offset: (scenario.width - agent.start.0 - 1) + agent.start.1,
            }
        } else {
            Diagonal {
                direction: DiagonalDirection::Down,
                half: DiagonalHalf::Right,
                offset: (scenario.width - agent.start.0 - 1) + agent.start.1,
            }
        };
        diagonals
            .entry(diagonal)
            .or_insert_with(Vec::new)
            .push(agent);
    }
    for (_, agents) in diagonals.iter_mut() {
        agents.sort_by_key(|agent| agent.start.0);
    }
    diagonals
}

#[cfg(test)]
mod tests {
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
        Grid::new(10, 10, vec![], (0, 0)),
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
}
