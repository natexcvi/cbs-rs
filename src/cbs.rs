use std::{collections::HashMap, error::Error, fmt, rc::Rc};

use clap::Parser;

use self::{
    high_level::{Agent, ConflictTreeNode, Constraint, Path},
    low_level::{AStarLowLevelSolver, Grid},
    search::a_star,
};

mod high_level;
pub(crate) mod io;
mod low_level;
mod mdd;
mod optimisations;
pub mod search;
mod vertex_cover;

#[derive(Debug, Clone)]
pub enum CBSError {
    AlreadySolved,
}

impl fmt::Display for CBSError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            CBSError::AlreadySolved => write!(f, "CBS instance already solved"),
        }
    }
}

impl Error for CBSError {}

#[derive(Parser, Debug, Clone)]
pub enum HighLevelHeuristic {
    ZeroHeuristic,
    DGHeuristic,
}

impl From<String> for HighLevelHeuristic {
    fn from(value: String) -> Self {
        match value.as_str() {
            "zero" => HighLevelHeuristic::ZeroHeuristic,
            "dg" => HighLevelHeuristic::DGHeuristic,
            _ => panic!("Invalid high level heuristic"),
        }
    }
}

pub struct DiagonalSubsolverConfig {
    slackness: i32,
    promotion_enabled: bool,
}

impl DiagonalSubsolverConfig {
    pub fn new(slackness: i32, promotion_enabled: bool) -> Self {
        Self {
            slackness,
            promotion_enabled,
        }
    }
}

pub struct CBSOptimisationConfig {
    priotising_conflicts: bool,
    bypassing_conflicts: bool,
    diagonal_subsolver: Option<DiagonalSubsolverConfig>,
    conflict_avoidance_table: bool,
    heuristic: HighLevelHeuristic,
}

impl CBSOptimisationConfig {
    pub fn new(
        priotising_conflicts: bool,
        bypassing_conflicts: bool,
        diagonal_subsolver: Option<DiagonalSubsolverConfig>,
        conflict_avoidance_table: bool,
        high_level_heuristic: Option<HighLevelHeuristic>,
    ) -> Self {
        CBSOptimisationConfig {
            priotising_conflicts,
            bypassing_conflicts,
            diagonal_subsolver,
            conflict_avoidance_table,
            heuristic: high_level_heuristic.unwrap_or(HighLevelHeuristic::ZeroHeuristic),
        }
    }
}

pub struct CBSInstance {
    map: Grid,
    agents: Vec<Agent>,
}

pub struct CBS {
    instance: CBSInstance,
    solved: bool,
    pub high_level_generated: usize,
    pub low_level_generated: usize,
    optimisation_config: CBSOptimisationConfig,
}

impl CBS {
    pub fn new(instance: CBSInstance, optimisation_config: Option<CBSOptimisationConfig>) -> Self {
        CBS {
            instance,
            high_level_generated: 0,
            low_level_generated: 0,
            solved: false,
            optimisation_config: optimisation_config
                .unwrap_or(CBSOptimisationConfig::new(false, false, None, false, None)),
        }
    }

    pub fn solve(&mut self) -> Result<HashMap<&Agent, Path>, Box<dyn Error>> {
        if self.solved {
            return Err(Box::new(CBSError::AlreadySolved));
        }
        let low_level_solver = AStarLowLevelSolver::new();
        let root = ConflictTreeNode::new(
            self.instance.agents.iter().collect(),
            Vec::<Box<Constraint>>::new(),
            HashMap::<&Agent, Vec<(i32, i32)>>::new(),
            &self.instance.map,
            if self.optimisation_config.priotising_conflicts {
                Some(optimisations::conflict_prioritisation::pick_conflict)
            } else {
                None
            },
            if self.optimisation_config.bypassing_conflicts {
                Some(optimisations::conflict_bypassing::bypass_conflict)
            } else {
                None
            },
            if let Some(subsolver_config) = &self.optimisation_config.diagonal_subsolver {
                Some(Rc::new(
                    optimisations::diagonal_subsolver::DiagonalSubsolver::new(
                        subsolver_config.slackness,
                        subsolver_config.promotion_enabled,
                    ),
                ))
            } else {
                None
            },
            self.optimisation_config.conflict_avoidance_table,
            &low_level_solver,
            match self.optimisation_config.heuristic {
                HighLevelHeuristic::ZeroHeuristic => {
                    Rc::new(high_level::heuristic::ZeroHeuristic::new())
                }
                HighLevelHeuristic::DGHeuristic => {
                    Rc::new(high_level::heuristic::DGHeuristic::new())
                }
            },
        );
        let solution = a_star(root);
        self.solved = true;
        match solution {
            Ok(solution) => {
                self.high_level_generated += solution.nodes_generated as usize;
                let last_node = solution.path.last().unwrap();
                let mut paths = HashMap::<&Agent, Path>::new();
                for agent in self.instance.agents.iter() {
                    paths.insert(agent, last_node.paths[agent].clone());
                }
                Ok(paths)
            }
            Err(error) => Err(Box::new(error)),
        }
    }
}

#[cfg(test)]
mod tests;
