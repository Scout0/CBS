"""
AStar Best First Search
"""

class AStar():
    def __init__(self, env):
        self.agent_dict = env.agent_dict
        self.admissible_heuristic = env.admissible_heuristic
        self.is_at_goal = env.is_at_goal
        self.find_neighbors = env.find_neighbors

    def recon_path(self, prev, curr):
        total_path = [curr]
        while curr in prev.keys():
            curr = prev[curr]
            total_path.append(curr)
        return total_path[::-1]

    def find(self, agent_name):
        """
        low level search 
        """
        initial_state = self.agent_dict[agent_name]["start"]
        step_cost = 1
        
        closed_set = set()
        open_set = {initial_state}

        prev = {}

        g_score = {} 
        g_score[initial_state] = 0

        f_score = {} 

        f_score[initial_state] = self.admissible_heuristic(initial_state, agent_name)

        while open_set:
            temp = {open_item:f_score.setdefault(open_item, float("inf")) for open_item in open_set}
            curr = min(temp, key=temp.get)

            if self.is_at_goal(curr, agent_name):
                return self.recon_path(prev, curr)

            open_set -= {curr}
            closed_set |= {curr}

            neighbor_list = self.find_neighbors(curr)

            for neighbor in neighbor_list:
                if neighbor in closed_set:
                    continue
                
                tentative_g_score = g_score.setdefault(curr, float("inf")) + step_cost

                if neighbor not in open_set:
                    open_set |= {neighbor}
                elif tentative_g_score >= g_score.setdefault(neighbor, float("inf")):
                    continue

                prev[neighbor] = curr

                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + self.admissible_heuristic(neighbor, agent_name)
        return False

