use std::{collections::HashMap, error::Error, fmt};

use self::{
    high_level::{Agent, ConflictTreeNode, Constraint, Path},
    low_level::{AStarLowLevelSolver, Grid},
    search::a_star,
};

mod high_level;
pub(crate) mod io;
mod low_level;
mod optimisations;
pub mod search;

#[derive(Debug)]
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

pub struct CBSOptimisationConfig {
    priotising_conflicts: bool,
    bypassing_conflicts: bool,
    diagonal_subsolver: bool,
    conflict_avoidance_table: bool,
}

impl CBSOptimisationConfig {
    pub fn new(
        priotising_conflicts: bool,
        bypassing_conflicts: bool,
        two_direction_subsolver: bool,
        conflict_avoidance_table: bool,
    ) -> Self {
        CBSOptimisationConfig {
            priotising_conflicts,
            bypassing_conflicts,
            diagonal_subsolver: two_direction_subsolver,
            conflict_avoidance_table,
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
                .unwrap_or(CBSOptimisationConfig::new(false, false, false, false)),
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
            if self.optimisation_config.diagonal_subsolver {
                Some(optimisations::diagonal_subsolver::plan_two_direction_agents)
            } else {
                None
            },
            self.optimisation_config.conflict_avoidance_table,
            &low_level_solver,
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
