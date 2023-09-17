use std::collections::{HashMap, HashSet};
use std::hash::Hash;
use std::rc::Rc;

use crate::cbs::high_level::{CTNodePreprocessor, ConflictTreeNode, Path};
use crate::cbs::low_level::{Grid, LocationTime};
use crate::cbs::search::dfs;
use crate::cbs::vertex_cover::{k_vertex_cover, min_vertex_cover, MVCGraph};
use crate::cbs::Agent;

pub(crate) struct DiagonalSubsolver {
    slackness: i32,
}

impl DiagonalSubsolver {
    pub(crate) fn new(slackness: i32) -> Self {
        Self { slackness }
    }
}

impl CTNodePreprocessor for DiagonalSubsolver {
    fn preprocess(&self, node: &mut ConflictTreeNode) {
        plan_two_direction_agents(node, self.slackness)
    }
}

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
    /// Returns the direction vecs of this [`Diagonal`].
    fn direction_vecs(&self) -> Vec<(i32, i32)> {
        match self.direction {
            DiagonalDirection::Up => match self.half {
                DiagonalHalf::Left => vec![(-1, 0), (0, -1), (0, 0)],
                DiagonalHalf::Right => vec![(1, 0), (0, 1), (0, 0)],
            },
            DiagonalDirection::Down => match self.half {
                DiagonalHalf::Left => vec![(0, 1), (-1, 0), (0, 0)],
                DiagonalHalf::Right => vec![(0, -1), (1, 0), (0, 0)],
            },
        }
    }
}

/// Plans the paths of the agents in the given [`ConflictTreeNode`] using the two-direction
/// subsolver.
/// Agents with no individually optimal paths are left unplanned for.
pub fn plan_two_direction_agents(node: &mut ConflictTreeNode, slackness: i32) {
    let diagonals = find_diagonal_sets(node.agents.iter(), &node.scenario);

    let diagonal_kinds = vec![
        (DiagonalDirection::Up, DiagonalHalf::Left),
        (DiagonalDirection::Up, DiagonalHalf::Right),
        (DiagonalDirection::Down, DiagonalHalf::Left),
        (DiagonalDirection::Down, DiagonalHalf::Right),
    ];

    diagonal_kinds.iter().for_each(|(direction, half)| {
        let mut chosen_diagonals = diagonals
            .iter()
            .filter(|(diagonal, _)| diagonal.direction == *direction && diagonal.half == *half)
            .collect::<Vec<(&Diagonal, &Vec<&Agent>)>>();
        chosen_diagonals.sort_by_key(
            |(diagonal, _)| match (&diagonal.direction, &diagonal.half) {
                (DiagonalDirection::Up, DiagonalHalf::Left) => diagonal.offset,
                (DiagonalDirection::Up, DiagonalHalf::Right) => -diagonal.offset,
                (DiagonalDirection::Down, DiagonalHalf::Left) => -diagonal.offset,
                (DiagonalDirection::Down, DiagonalHalf::Right) => diagonal.offset,
            },
        );
        plan_diagonal_kind(node, chosen_diagonals, slackness);
    });
}

fn plan_diagonal_kind<'a, 'b>(
    node: &'a mut ConflictTreeNode<'b>,
    diagonals: Vec<(&Diagonal, &Vec<&'b Agent>)>,
    slackness: i32,
) where
    'b: 'a,
{
    let mut aux_grid = Grid::new(
        node.scenario.width,
        node.scenario.height,
        node.scenario.obstacles.clone().into_iter().collect(),
        node.scenario.goal,
    );
    let mut promoted_agents = Vec::<&Agent>::new();
    for (diagonal, agents) in diagonals.iter() {
        let mut paths = HashMap::<&Agent, Path>::new();
        let mut target_obstacles = Vec::<LocationTime>::new();
        let mut planned_path_obstacles = HashSet::<LocationTime>::new();
        let augmented_agents = agents
            .clone()
            .to_owned()
            .into_iter()
            .chain(promoted_agents.clone().into_iter())
            .collect::<Vec<_>>();
        let dependency_graph = build_dependency_graph(&augmented_agents);
        let agents_to_promote = min_vertex_cover(&dependency_graph);
        for agent in augmented_agents.iter() {
            if agents_to_promote.contains(&Rc::new(*agent)) {
                promoted_agents.push(agent);
                continue;
            }
            let mut constraint_obstacles = node.constraints_to_obstacles(agent);
            constraint_obstacles.extend(Grid::to_conditional_obstacles(
                planned_path_obstacles.clone().into_iter().collect(),
            ));
            let (path, found) =
                plan_agent_path(agent, diagonal, &aux_grid, &constraint_obstacles, slackness);
            if !found {
                continue;
            }
            path.iter().enumerate().for_each(|(i, loc)| {
                planned_path_obstacles.insert(LocationTime {
                    location: loc.clone(),
                    time: i as i32,
                });
            });
            paths.insert(agent, path);
            target_obstacles.push(LocationTime {
                location: agent.goal.clone(),
                time: -1,
            });
        }
        promoted_agents.retain(|agent| !paths.contains_key(agent));
        node.paths.extend(paths);
        aux_grid
            .obstacles
            .extend(Grid::to_conditional_obstacles(target_obstacles));
    }
}

fn plan_agent_path(
    agent: &&Agent,
    diagonal: &Diagonal,
    aux_grid: &Grid,
    additional_obstacles: &HashMap<LocationTime, Vec<(i32, i32)>>,
    slackness: i32,
) -> (Vec<(i32, i32)>, bool) {
    let latest_goal_obstacle_time = additional_obstacles
        .iter()
        .filter(|(loc_time, coming_from)| loc_time.location == agent.goal && coming_from.is_empty())
        .map(|(loc, _)| loc.time)
        .max()
        .unwrap_or(i32::MIN); // TODO: think if aux_grid obstacles should be considered here
    let mut visited = HashSet::<LocationTime>::new();
    let mut path: Path = vec![];
    let found = dfs(
        &mut visited,
        &mut path,
        &|path, cur, prev| {
            path.push(cur.location.clone());
            if let Some(prev) = prev {
                if cur.location == prev.location
                    && (slackness == 0 || max_waits_exceeded(path, slackness))
                {
                    return false;
                }
            }
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
                    aux_grid.is_valid_location_time(&loc, &cur.location)
                        && is_in_start_goal_box(loc, agent)
                        && !is_in_additional_obstacles(loc, cur, additional_obstacles)
                })
                .collect()
        },
        &|cur| cur.location == agent.goal,
    );
    if path.len() as i32 <= latest_goal_obstacle_time {
        return (path, false);
    }
    (path, found)
}

fn max_waits_exceeded(path: &Path, max_waits_allowed: i32) -> bool {
    let mut num_waits = 0;
    for i in 1..path.len() {
        if path[i] == path[i - 1] {
            num_waits += 1;
            if num_waits > max_waits_allowed {
                return true;
            }
        }
    }
    false
}

/// Determines if the two agents, that must come from the same diagonal,
/// are dependent in the sense that they must cross each other's path.
fn are_dependent(agent: &Agent, other_agent: &Agent) -> bool {
    let (x, y) = agent.start;
    let (other_x, other_y) = other_agent.start;
    let (goal_x, goal_y) = agent.goal;
    let (other_goal_x, other_goal_y) = other_agent.goal;
    (x - other_x) * (goal_x - other_goal_x) < 0 && (y - other_y) * (goal_y - other_goal_y) < 0
}

fn build_dependency_graph<'a>(agents: &'_ Vec<&'a Agent>) -> MVCGraph<&'a Agent> {
    let mut graph = MVCGraph::new();
    for agent in agents.iter() {
        for other_agent in agents.iter() {
            if are_dependent(agent, other_agent) {
                graph.add_edge(Rc::new(*agent), Rc::new(*other_agent));
            }
        }
    }
    graph
}

fn is_in_additional_obstacles(
    loc_time: &LocationTime,
    prev: &LocationTime,
    additional_obstacles: &HashMap<LocationTime, Vec<(i32, i32)>>,
) -> bool {
    match additional_obstacles.get(loc_time) {
        Some(coming_from) => coming_from.is_empty() || coming_from.contains(&prev.location),
        None => false,
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
    for (diag, agents) in diagonals.iter_mut() {
        agents.sort_by_key(|agent| match (&diag.direction, &diag.half) {
            (DiagonalDirection::Up, DiagonalHalf::Left) => agent.start.0,
            (DiagonalDirection::Up, DiagonalHalf::Right) => -agent.start.0,
            (DiagonalDirection::Down, DiagonalHalf::Left) => -agent.start.0,
            (DiagonalDirection::Down, DiagonalHalf::Right) => agent.start.0,
        });
    }
    diagonals
}

#[cfg(test)]
mod tests;
