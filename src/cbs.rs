use std::{collections::HashMap, error::Error, fmt};

use self::{
    high_level::{Agent, ConflictTreeNode, Constraint, Path},
    low_level::Grid,
    search::a_star,
};

mod high_level;
mod low_level;
mod optimisations;
pub mod search;
mod io;

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
}

pub struct CBS<'a> {
    scenario: &'a Grid,
    agents: Vec<&'a Agent>,
    solved: bool,
    pub high_level_expanded: usize,
    pub low_level_expanded: usize,
    optimisation_config: CBSOptimisationConfig,
}

impl<'a> CBS<'a> {
    pub fn new(
        scenario: &'a Grid,
        agents: Vec<&'a Agent>,
        optimisation_config: Option<CBSOptimisationConfig>,
    ) -> Self {
        CBS {
            scenario,
            agents,
            high_level_expanded: 0,
            low_level_expanded: 0,
            solved: false,
            optimisation_config: optimisation_config.unwrap_or(CBSOptimisationConfig {
                priotising_conflicts: false,
                bypassing_conflicts: false,
            }),
        }
    }

    pub fn solve(&mut self) -> Result<HashMap<&Agent, Path>, Box<dyn Error>> {
        if self.solved {
            return Err(Box::new(CBSError::AlreadySolved));
        }
        let root = ConflictTreeNode::new(
            self.agents.clone(),
            Vec::<Box<Constraint>>::new(),
            HashMap::<&Agent, Vec<(i32, i32)>>::new(),
            self.scenario,
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
        );
        let solution = a_star(root);
        self.solved = true;
        match solution {
            Ok(solution) => {
                self.high_level_expanded += solution.nodes_expanded as usize;
                let last_node = solution.path.last().unwrap();
                let mut paths = HashMap::<&Agent, Path>::new();
                for agent in self.agents.iter() {
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
