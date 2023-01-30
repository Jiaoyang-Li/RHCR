#include "KivaSystem.h"
#include "SortingSystem.h"
#include "OnlineSystem.h"
#include "BeeSystem.h"
#include "ID.h"
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>


void set_parameters(BasicSystem& system, const boost::program_options::variables_map& vm)
{
	system.outfile = vm["output"].as<std::string>();
	system.screen = vm["screen"].as<int>();
	system.log = vm["log"].as<bool>();
	system.num_of_drives = vm["agentNum"].as<int>();
	system.time_limit = vm["cutoffTime"].as<int>();
	system.simulation_window = vm["simulation_window"].as<int>();
	system.planning_window = vm["planning_window"].as<int>();
	system.travel_time_window = vm["travel_time_window"].as<int>();
	system.consider_rotation = vm["rotation"].as<bool>();
	system.k_robust = vm["robust"].as<int>();
	system.hold_endpoints = vm["hold_endpoints"].as<bool>();
	system.useDummyPaths = vm["dummy_paths"].as<bool>();
	if (vm.count("seed"))
		system.seed = vm["seed"].as<int>();
	else
		system.seed = (int)time(0);
	srand(system.seed);
}


MAPFSolver* set_solver(const BasicGraph& G, const boost::program_options::variables_map& vm)
{
	string solver_name = vm["single_agent_solver"].as<string>();
	SingleAgentSolver* path_planner;
	MAPFSolver* mapf_solver;
	if (solver_name == "ASTAR")
	{
		path_planner = new StateTimeAStar();
	}
	else if (solver_name == "SIPP")
	{
		path_planner = new SIPP();
	}
	else
	{
		cout << "Single-agent solver " << solver_name << "does not exist!" << endl;
		exit(-1);
	}

	solver_name = vm["solver"].as<string>();
	if (solver_name == "ECBS")
	{
		ECBS* ecbs = new ECBS(G, *path_planner);
		ecbs->potential_function = vm["potential_function"].as<string>();
		ecbs->potential_threshold = vm["potential_threshold"].as<double>();
		ecbs->suboptimal_bound = vm["suboptimal_bound"].as<double>();
		mapf_solver = ecbs;
	}
	else if (solver_name == "PBS")
	{
		PBS* pbs = new PBS(G, *path_planner);
		pbs->lazyPriority = vm["lazyP"].as<bool>();
        auto prioritize_start = vm["prioritize_start"].as<bool>();
        if (vm["hold_endpoints"].as<bool>() or vm["dummy_paths"].as<bool>())
            prioritize_start = false;
        pbs->prioritize_start = prioritize_start;
        pbs->setRT(vm["CAT"].as<bool>(), prioritize_start);
		mapf_solver = pbs;
	}
	else if (solver_name == "WHCA")
	{
		mapf_solver = new WHCAStar(G, *path_planner);
	}
	else if (solver_name == "LRA")
	{
		mapf_solver = new LRAStar(G, *path_planner);
	}
	else
	{
		cout << "Solver " << solver_name << "does not exist!" << endl;
		exit(-1);
	}

	if (vm["id"].as<bool>())
	{
		return new ID(G, *path_planner, *mapf_solver);
	}
	else
	{
		return mapf_solver;
	}
}


int main(int argc, char** argv) 
{
	namespace po = boost::program_options;
	// Declare the supported options.
	po::options_description desc("Allowed options");
	desc.add_options()
		("help", "produce help message")
		("scenario", po::value<std::string>()->required(), "scenario (SORTING, KIVA, ONLINE, BEE)")
		("map,m", po::value<std::string>()->required(), "input map file")
		("task", po::value<std::string>()->default_value(""), "input task file")
		("output,o", po::value<std::string>()->default_value("../exp/test"), "output folder name")
		("agentNum,k", po::value<int>()->required(), "number of drives")
		("cutoffTime,t", po::value<int>()->default_value(60), "cutoff time (seconds)")
		("seed,d", po::value<int>(), "random seed")
		("screen,s", po::value<int>()->default_value(1), "screen option (0: none; 1: results; 2:all)")
		("solver", po::value<string>()->default_value("PBS"), "solver (LRA, PBS, WHCA, ECBS)")
		("id", po::value<bool>()->default_value(false), "independence detection")
		("single_agent_solver", po::value<string>()->default_value("SIPP"), "single-agent solver (ASTAR, SIPP)")
		("lazyP", po::value<bool>()->default_value(false), "use lazy priority")
		("simulation_time", po::value<int>()->default_value(5000), "run simulation")
		("simulation_window", po::value<int>()->default_value(5), "call the planner every simulation_window timesteps")
		("travel_time_window", po::value<int>()->default_value(0), "consider the traffic jams within the given window")
		("planning_window", po::value<int>()->default_value(INT_MAX / 2),
		        "the planner outputs plans with first planning_window timesteps collision-free")
		("potential_function", po::value<string>()->default_value("NONE"), "potential function (NONE, SOC, IC)")
		("potential_threshold", po::value<double>()->default_value(0), "potential threshold")
		("rotation", po::value<bool>()->default_value(false), "consider rotation")
		("robust", po::value<int>()->default_value(0), "k-robust (for now, only work for PBS)")
		("CAT", po::value<bool>()->default_value(false), "use conflict-avoidance table")
		// ("PG", po::value<bool>()->default_value(false),
		//        "reuse the priority graph of the goal node of the previous search")
		("hold_endpoints", po::value<bool>()->default_value(false),
		        "Hold endpoints from Ma et al, AAMAS 2017")
		("dummy_paths", po::value<bool>()->default_value(false),
				"Find dummy paths from Liu et al, AAMAS 2019")
		("prioritize_start", po::value<bool>()->default_value(true), "Prioritize waiting at start locations")
		("suboptimal_bound", po::value<double>()->default_value(1), "Suboptimal bound for ECBS")
		("log", po::value<bool>()->default_value(false), "save the search trees (and the priority trees)")
		;
	clock_t start_time = clock();
	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);

	if (vm.count("help")) {
		std::cout << desc << std::endl;
		return 1;
	}

	po::notify(vm);

    // check params
    if (vm["hold_endpoints"].as<bool>() or vm["dummy_paths"].as<bool>())
    {
        if (vm["hold_endpoints"].as<bool>() and vm["dummy_paths"].as<bool>())
        {
            std::cerr << "Hold endpoints and dummy paths cannot be used simultaneously" << endl;
            exit(-1);
        }
        if (vm["simulation_window"].as<int>() != 1)
        {
            std::cerr << "Hold endpoints and dummy paths can only work when the simulation window is 1" << endl;
            exit(-1);
        }
        if (vm["planning_window"].as<int>() < INT_MAX / 2)
        {
            std::cerr << "Hold endpoints and dummy paths cannot work with planning windows" << endl;
            exit(-1);
        }
    }

    // make dictionary
	boost::filesystem::path dir(vm["output"].as<std::string>() +"/");
	boost::filesystem::create_directories(dir);
	if (vm["log"].as<bool>())
	{
		boost::filesystem::path dir1(vm["output"].as<std::string>() + "/goal_nodes/");
		boost::filesystem::path dir2(vm["output"].as<std::string>() + "/search_trees/");
		boost::filesystem::create_directories(dir1);
		boost::filesystem::create_directories(dir2);
	}


	if (vm["scenario"].as<string>() == "KIVA")
	{
		KivaGrid G;
		if (!G.load_map(vm["map"].as<std::string>()))
			return -1;
		MAPFSolver* solver = set_solver(G, vm);
		KivaSystem system(G, *solver);
		set_parameters(system, vm);
		G.preprocessing(system.consider_rotation);
		system.simulate(vm["simulation_time"].as<int>());
		return 0;
	}
	else if (vm["scenario"].as<string>() == "SORTING")
	{
		 SortingGrid G;
		 if (!G.load_map(vm["map"].as<std::string>()))
			 return -1;
		 MAPFSolver* solver = set_solver(G, vm);
		 SortingSystem system(G, *solver);
		 assert(!system.hold_endpoints);
		 assert(!system.useDummyPaths);
		 set_parameters(system, vm);
		 G.preprocessing(system.consider_rotation);
		 system.simulate(vm["simulation_time"].as<int>());
		 return 0;
	}
	else if (vm["scenario"].as<string>() == "ONLINE")
	{
		OnlineGrid G;
		if (!G.load_map(vm["map"].as<std::string>()))
			return -1;
		MAPFSolver* solver = set_solver(G, vm);
		OnlineSystem system(G, *solver);
		assert(!system.hold_endpoints);
		assert(!system.useDummyPaths);
		set_parameters(system, vm);
		G.preprocessing(system.consider_rotation);
		system.simulate(vm["simulation_time"].as<int>());
		return 0;
	}
	else if (vm["scenario"].as<string>() == "BEE")
	{
		BeeGraph G;
		if (!G.load_map(vm["map"].as<std::string>()))
			return -1;
		MAPFSolver* solver = set_solver(G, vm);
		BeeSystem system(G, *solver);
		assert(!system.hold_endpoints);
		assert(!system.useDummyPaths);
		set_parameters(system, vm);
		G.preprocessing(vm["task"].as<std::string>(), system.consider_rotation);
		system.load_task_assignments(vm["task"].as<std::string>());
		system.simulate();
		double runtime = (double)(clock() - start_time)/ CLOCKS_PER_SEC;
		cout << "Overall runtime:			" << runtime << " seconds." << endl;
		// cout << "	Reading from file:		" << G.loading_time + system.loading_time << " seconds." << endl;
		// cout << "	Preprocessing:			" << G.preprocessing_time << " seconds." << endl;
		// cout << "	Writing to file:		" << system.saving_time << " seconds." << endl;
		cout << "Makespan:		" << system.get_makespan() << " timesteps." << endl;
		cout << "Flowtime:		" << system.get_flowtime() << " timesteps." << endl;
		cout << "Flowtime lowerbound:	" << system.get_flowtime_lowerbound() << " timesteps." << endl;
		auto flower_ids = system.get_missed_flower_ids();
		cout << "Missed tasks:";
		for (auto id : flower_ids)
			cout << " " << id;
		cout << endl;
		// cout << "Remaining tasks: " << system.get_num_of_remaining_tasks() << endl;
		cout << "Objective: " << system.get_objective() << endl;
		std::ofstream output;
		output.open(vm["output"].as<std::string>() + "/MAPF_results.txt", std::ios::out);
		output << "Overall runtime: " << runtime << " seconds." << endl;;
		output << "Makespan: " << system.get_makespan() << " timesteps." << endl;
		output << "Flowtime: " << system.get_flowtime() << " timesteps." << endl;
		output << "Flowtime lowerbound: " << system.get_flowtime_lowerbound() << " timesteps." << endl;
		output << "Missed tasks:";
		for (auto id : flower_ids)
			output << " " << id;
		output << endl;
		output << "Objective: " << system.get_objective() << endl;
		output.close();
        return 0;
	}
	else
	{
		cout << "Scenario " << vm["scenario"].as<string>() << "does not exist!" << endl;
		return -1;
	}
}
