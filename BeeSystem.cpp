#include "BeeSystem.h"
#include <boost/tokenizer.hpp>


BeeSystem::BeeSystem(const BeeGraph& G, MAPFSolver& solver) : BasicSystem(G, solver), G(G) {}


BeeSystem::~BeeSystem()
{
}

bool BeeSystem::load_task_assignments(string fname)
{
	clock_t t = clock();
	using namespace boost;
	string line;
	std::ifstream myfile((fname).c_str());
	if (!myfile.is_open())
	{
		return false;
	}
	char_separator<char> sep(" ");
	int id, i;
	char temp;
	task_sequences.resize(G.initial_locations.size());
	while (getline(myfile, line)) 
	{
		std::istringstream iss(line);
		iss >> i >> temp;
		if (i == 0)
		{
			task_sequences.emplace_back();
			while (iss >> id)
			{
				task_sequences.back().emplace_back(G.flowers[id - 1], G.flower_time_windows[id - 1].first);
			}
			task_sequences.back().emplace_back(G.entrance, 0);
		}
		else
		{
			while (iss >> id)
			{
				task_sequences[i - 1].emplace_back(G.flowers[id - 1], G.flower_time_windows[id - 1].first);
			}
			task_sequences[i - 1].emplace_back(G.entrance, 0);
		}
		
	}
	myfile.close();
	loading_time = (clock() - t) * 1.0 / CLOCKS_PER_SEC;
	return true;
}


void BeeSystem::initialize()
{
	initialize_solvers();

	starts.resize(num_of_drives);
	goal_locations.resize(num_of_drives);
	paths.resize(num_of_drives);
	finished_tasks.resize(num_of_drives);
	timestep = 0;
	initialize_start_locations();
}

void BeeSystem::initialize_start_locations()
{
	for (int k = 0; k < (int)G.initial_locations.size(); k++)
	{
		int orientation = -1;
		if (consider_rotation)
		{
			orientation = rand() % 4;
		}
		int loc = G.initial_locations[k];
		starts[k] = State(loc, 0, orientation);
		paths[k].emplace_back(starts[k]);
		finished_tasks[k].emplace_back(loc, 0);
	}
	for (int k = (int)G.initial_locations.size(); k < num_of_drives; k++)
	{
		int orientation = -1;
		if (consider_rotation)
		{
			orientation = rand() % 4;
		}
		int loc = G.entrance;
		starts[k] = State(loc, 0, orientation);
		paths[k].emplace_back(starts[k]);
		finished_tasks[k].emplace_back(loc, 0);
	}
}


void BeeSystem::update_goal_locations()
{
	for (int k = 0; k < num_of_drives; k++)
	{
		pair<int, int> curr(paths[k][timestep].location, timestep); // current location

		pair<int, int> goal; // The last goal location
		if (goal_locations[k].empty())
		{
			goal = curr;
		}
		else
		{
			goal = goal_locations[k].back();
		}
		int min_timesteps = max(G.get_Manhattan_distance(curr.first, goal.first), // cannot use h values, because graph edges may have weights
			goal.second - timestep); 
		while (min_timesteps <= simulation_window)
			// The agent might finish its tasks during the next planning horizon
		{
			// assign a new task
			if (task_sequences.size() <= k) // no more task sequences for this agent
				break;
			else if (task_sequences[k].empty()) // the current path has been finished. Pick the next task sequence
			{
				if (task_sequences.size() <= num_of_drives) // no more task sequences for this agent
					break;
				task_sequences[k] = task_sequences[num_of_drives];
				task_sequences.erase(task_sequences.begin() + num_of_drives);
			}
			auto next = task_sequences[k].front();
			task_sequences[k].pop_front();
			goal_locations[k].emplace_back(next);
			min_timesteps += G.get_Manhattan_distance(next.first, goal.first); 
			min_timesteps = max(min_timesteps, goal.second - timestep);
			goal = next;
		}
		if (goal_locations[k].empty())
		{
			goal_locations[k].emplace_back(G.entrance, 0);
		}
	}
}


void BeeSystem::simulate()
{
	if (screen > 0)
		std::cout << "*** Simulating " << seed << " ***" << std::endl;
	this->simulation_time = G.max_timestep;
	this->num_of_drives = G.num_of_bees;
	initialize();

	for (; timestep < simulation_time; timestep += simulation_window)
	{
		if (screen > 0)
			std::cout << "Timestep " << timestep << std::endl;

		update_start_locations();
		update_goal_locations();
		solve();

		// move drives
		auto new_finished_tasks = move();
		int old = num_of_tasks;
		// update tasks
		for (auto task : new_finished_tasks)
		{
			int id, loc, t;
			std::tie(id, loc, t) = task;
			if (loc != G.entrance || finished_tasks[id].back().first != G.entrance)
			{
				finished_tasks[id].emplace_back(loc, t);
				if (loc != G.entrance)
					num_of_tasks++;
			}
		}
		if (screen > 0)
		{
			std::cout << num_of_tasks - old << " tasks just finished" << std::endl;
			std::cout << num_of_tasks << " tasks finished in total" << std::endl;
			std::cout << task_sequences.size() << " paths remain" << std::endl;
		}

		bool stop = true;
		for (const auto goals : goal_locations)
		{
			if (!goals.empty())
			{
				stop = false;
				break;
			}
		}
		if (stop)
		{
			for (const auto tasks : task_sequences)
			{
				if (!tasks.empty())
				{
					stop = false;
					break;
				}
			}
		}
		if (stop)
		{
			timestep += simulation_window;
			break;
		}
	}

	update_start_locations();
	if (screen)
	{
		std::cout << std::endl << "Done!" << std::endl;
		cout << num_of_tasks << " tasks have been completed" << endl;
	}
	save_results();
}

int BeeSystem::get_num_of_missed_tasks() const
{
	int missed_tasks = 0;
	for (auto finished_task : finished_tasks)
	{
		for (auto task : finished_task)
		{
			for (int i = 0; i < (int)G.flowers.size(); i++)
			{
				if (task.first == G.flowers[i] && task.second > G.flower_time_windows[i].second)
				{
					missed_tasks++;
					break;
				}
			}
		}
	}
	return missed_tasks;
}

int BeeSystem::get_num_of_remaining_tasks() const
{
	int remaining_tasks = 0;
	for (const auto& tasks : task_sequences)
	{
		remaining_tasks += (int) tasks.size();
	}
	for (const auto& tasks : finished_tasks)
	{
		auto task = tasks.rbegin();
		while (task != tasks.rend() && task->first != G.entrance) // robot fails to carry the items back to the entrance
		{
			remaining_tasks++;
			task++;
		}
	}
	return remaining_tasks;
}

list<int> BeeSystem::get_missed_flower_ids() const
{
	list<int> ids;
	for (const auto& tasks : task_sequences) // unfinished tasks
	{
		for (const auto& task : tasks)
		{
			for (int i = 0; i < (int)G.flowers.size(); i++)
			{
				if (task.first == G.flowers[i])
				{
					ids.push_back(i + 1);
					break;
				}
			}
		}
	}
	for (auto finished_task : finished_tasks) // finished tasks
	{
		auto task = finished_task.rbegin();
		while (task != finished_task.rend() && task->first != G.entrance) // robot fails to carry the items back to the entrance
		{
			for (int i = 0; i < (int)G.flowers.size(); i++)
			{
				if (task->first == G.flowers[i])
				{
					ids.push_back(i + 1);
					break;
				}
			}
			++task;
		}
		while (task != finished_task.rend()) // robot reaches the task after its deadline
		{
			for (int i = 0; i < (int)G.flowers.size(); i++)
			{
				if (task->first == G.flowers[i])
				{
					if (task->second > G.flower_time_windows[i].second)
						ids.push_back(i + 1);
					break;
				}
			}
			++task;
		}
	}
	return ids;
}

int BeeSystem::get_makespan()
{
	while (paths[0].size() > 1)
	{
		int T = (int)paths[0].size() - 1;
		for (const auto& path : paths)
		{
			if (path[T - 1].location != G.entrance)
			{
				return T;
			}
		}
		for (int i = 0; i < int(paths.size()); i++)
		{
			paths[i].pop_back();
		}
	}
	return -1; // should never happen
}

int BeeSystem::get_flowtime() const
{
	int flowtime = 0;
	for (const auto& path : paths)
	{
		for (int t = (int)path.size() - 1; t > 0; t--)
		{
			if (path[t - 1].location != G.entrance)
			{
				flowtime += t;
				break;
			}
		}
	}
	return flowtime;
}

int BeeSystem::get_flowtime_lowerbound() const
{
	int rst = 0;
	for (int k = 0; k < num_of_drives; k++)
	{
		int prev = finished_tasks[k].front().first;
		for (auto task : finished_tasks[k])
		{
			if (task.second != 0)
				rst += G.heuristics.at(task.first)[prev];
			prev = task.first;
		}
	}
	return rst / G.move_cost;
}

int BeeSystem::get_objective() const
{
	int path_cost = 0;
	for (auto path : paths)
	{
		for (int t = 0; t < (int)path.size() - 1; t++)
		{
			path_cost += G.get_weight(path[t].location, path[t].location);
		}
	}
	int task_cost = 0;
	for (auto finished_task : finished_tasks)
	{
		auto task = finished_task.rbegin();
		while (task != finished_task.rend() && task->first != G.entrance) // robot fails to carry the items back to the entrance
		{
			++task;
		}
		while (task != finished_task.rend())
		{
			for (int i = 0; i < (int)G.flowers.size(); i++)
			{
				if (task->first == G.flowers[i] && task->second <= G.flower_time_windows[i].second)
				{
					task_cost += G.flower_costs[i];
					break;
				}
			}
			++task;
		}
	}
	return path_cost + task_cost;
}

void solve_VRP_by_LKH3(string fname)
{
	clock_t t = clock();
	std::size_t pos = fname.rfind('.');      // position of the file extension
	string input_file = fname.substr(0, pos);     // get the name without extension
	std::ofstream output;
	output.open(input_file + "_LKH_input.txt", std::ios::out);
	output << "NAME: BEE" << endl <<
						"TYPE: ";
	output.close();
}