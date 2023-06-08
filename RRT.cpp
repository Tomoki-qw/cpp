#pragma once

#include<vector>
#include<functional>
#include<string>
#include<memory>
//#include"scara_position.h"

//#define DEBUG_RRT
#ifdef DEBUG_RRT
//using T = point_t;
#else
#endif
template <typename T>

class RRT_t {
#ifdef DEBUG_RRT
public:
#endif
	std::function<double(const T&, const T&)> calc_dist_func;
	std::function<bool(const T&)> check_point_func;
	std::function<T()> random_point_func;
	bool check_line(const T& p0, const T& p1, double ds) {
		double s = 0;
		T diff = p1 - p0;
		double abs_diff = calc_dist_func(p0, p1);
		diff /= abs_diff;
		while (s < abs_diff) {
			if (!check_point_func(p0 + s * diff)) {
				return false;
			}
			s += ds;
		}
		return check_point_func(p1);
	}
public:
	class node_t {
		RRT_t* rrt;
		T point;
		int index;
		double distance;//start_nodeÇ©ÇÁÇÃãóó£
		std::weak_ptr<node_t> ptr_parent;
		std::vector<std::shared_ptr<node_t>> ptr_children;
	public:
		double get_distance()const { return distance; }
		void set_point(const T& p) { point = p; }
		T get_point()const { return point; }
		void set_rrt(RRT_t* rrt) { this->rrt = rrt; }
		double dist(std::shared_ptr<node_t> ptr_node) { return rrt->calc_dist_func(point, ptr_node->point); }
		node_t() :rrt(NULL), index(0), distance(0) {}
		node_t(std::shared_ptr<node_t> parent, const T& point, int index) :rrt(parent->rrt), point(point), index(index), ptr_parent(parent),
			distance(parent->distance + rrt->calc_dist_func(parent->point, point)) {

#ifdef DEBUG_RRT
			if (index < 0 || index > parent->ptr_children.size()) {
				std::cout << "Error@RRt::node_t::node_t(std::shared_ptr<node_t> parent, const T& point, int index) : index=" << index << " is wrong value" << std::endl;
			}
#endif
		}

		std::shared_ptr<node_t> get_self_ptr()const {
			if (ptr_parent.use_count()) {
				return ptr_parent.lock()->ptr_children[index];
			}
			else {
				return rrt->ptr_start_node;
			}
		}

		std::shared_ptr<node_t> add_child(const T& point) {
			ptr_children.push_back(std::make_shared<node_t>(get_self_ptr(), point, ptr_children.size()));
			ptr_children.back()->index = ptr_children.size() - 1;
			return ptr_children.back();
		}

		bool delete_child(int index_child) {
			if (index_child < 0 || index_child >= ptr_children.size()) {
				std::cout << "Error@RRt::node_t::delete_child(int index_child) : index_child=" << index_child << " is wrong value" << std::endl;
				return false;
			}
			ptr_children.erase(ptr_children.begin() + index_child);
			for (int i = index_child; i < ptr_children.size(); ++i) {
				--(ptr_children[i]->index);
				if (ptr_children[i]->index != i) {
					std::cout << "Error@RRt::node_t::delete_child(int index_child) : children->index is wrong value" << std::endl;
					return false;
				}
			}
			return true;
		}

		void delete_branch() {
			if (!ptr_parent.use_count())return;
			std::shared_ptr<node_t> parent = ptr_parent.lock();
			if (parent->ptr_children.size() == 1) {
				parent->delete_branch();
			}
			else {
				parent->delete_child(index);
			}
		}

		void get_end_children(std::vector<std::shared_ptr<node_t>>& end_children) {
			if (ptr_children.empty()) {
				end_children.push_back(get_self_ptr());
			}
			else {
				for (std::shared_ptr<node_t>& ptr_node : ptr_children) {
					ptr_node->get_end_children(end_children);
				}
			}
		}

		void get_all_children(std::vector<std::shared_ptr<node_t>>& all_children) {
			all_children.push_back(get_self_ptr());
			for (std::shared_ptr<node_t>& ptr_node : ptr_children) {
				ptr_node->get_all_children(all_children);
			}
		}

		void get_reverse_branch(std::vector<T>& reverse_branch)const {
			reverse_branch.push_back(point);
			if (ptr_parent.use_count())ptr_parent.lock()->get_reverse_branch(reverse_branch);
		}
	};
private:
#ifdef DEBUG_RRT
public:
#endif
	std::shared_ptr<node_t> ptr_start_node;
	void get_end_nodes(std::vector<std::shared_ptr<node_t>>& end_nodes)const {
		end_nodes.clear();
		ptr_start_node->get_end_children(end_nodes);
	}

	void get_all_nodes(std::vector<std::shared_ptr<node_t>>& all_nodes)const {
		all_nodes.clear();
		ptr_start_node->get_all_children(all_nodes);
	}

	void get_branch(std::shared_ptr<node_t>& node, std::vector<T>& branch)const {
		branch.clear();
		std::vector<T> reverse_branch;
		node->get_reverse_branch(reverse_branch);
		branch.resize(reverse_branch.size());
		for (int i = 0; i < reverse_branch.size(); ++i)branch[i] = reverse_branch[reverse_branch.size() - 1 - i];
	}

	void get_all_branch(std::vector<std::vector<T>>& all_branch)const {
		all_branch.clear();
		std::vector<std::shared_ptr<node_t>> end_nodes;
		get_end_nodes(end_nodes);
		all_branch.resize(end_nodes.size());
		for (int i = 0; i < end_nodes.size(); ++i) {
			get_branch(end_nodes[i], all_branch[i]);
		}
	}

	void set_start_point(const T& start_point) {
		ptr_start_node->set_point(start_point);
	}
	void add_new_point(double step_length, double ds, const T& random_point) {
		std::vector<std::shared_ptr<node_t>> all_nodes;
		get_all_nodes(all_nodes);
		double d_min = calc_dist_func(all_nodes[0]->get_point(), random_point);
		int i_d_min = 0;
		for (int i = 1; i < all_nodes.size(); ++i) {
			double d = calc_dist_func(all_nodes[i]->get_point(), random_point);
			if (d < d_min) {
				d_min = d;
				i_d_min = i;
			}
		}

		T d_min_point = all_nodes[i_d_min]->get_point();
		T diff = random_point - d_min_point;
		diff /= calc_dist_func(random_point, d_min_point);
		if (!check_line(d_min_point, d_min_point + diff * step_length, ds))return;

		std::shared_ptr<node_t> new_node = (all_nodes[i_d_min]->add_child(d_min_point + diff * step_length));

		//ëºÇÃåoòHÇÃç≈ìKâª
		std::vector<std::shared_ptr<node_t>> end_nodes;
		get_end_nodes(end_nodes);
		for (std::shared_ptr<node_t>& node : end_nodes) {
			if (node.get() == new_node.get()) continue;
			if (node->get_distance() > new_node->get_distance() + new_node->dist(node) &&
				check_line(node->get_point(), new_node->get_point(), ds))
			{
				new_node->add_child(node->get_point());
				node->delete_branch();
			}
		}

	}
public:
	void init(
		std::function<double(const T&, const T&)> calc_dist_func,
		std::function<bool(const T&)> check_point_func,
		std::function<T()> random_point_func
	) {
		this->calc_dist_func = calc_dist_func;
		this->check_point_func = check_point_func;
		this->random_point_func = random_point_func;
		ptr_start_node = std::make_shared<node_t>();
		ptr_start_node->set_rrt(this);
	}
	void generate_path(const T& start_point, const T& finish_point, double step_length, double ds, int N_cycle_min, std::vector<T>& path) {
		set_start_point(start_point);
		for (int i = 0; i < N_cycle_min; ++i) {
			add_new_point(step_length, ds, random_point_func());
		}

		std::vector<std::shared_ptr<node_t>> all_nodes;
		get_all_nodes(all_nodes);
		int i_d_min = -1;
		double d_min;
		while (1) {
			for (int i = 1; i < all_nodes.size(); ++i) {
				if (check_line(finish_point, all_nodes[i]->get_point(), ds)) {
					double d = calc_dist_func(finish_point, all_nodes[i]->get_point());
					if (i_d_min == -1) {
						i_d_min = i;
						d_min = d;
					}
					else if (d < d_min) {
						d_min = d;
						i_d_min = i;
					}
				}
			}
			if (i_d_min != -1) {
				break;
			}
			else {
				for (int i = 0; i < 10; ++i) add_new_point(step_length, ds, random_point_func());
			}
		}
		std::shared_ptr<node_t> finish_node = all_nodes[i_d_min]->add_child(finish_point);

		std::vector<T> path0;
		get_branch(finish_node, path0);

		//pathÇï‚äÆ
		path.clear();
		double ds0 = ds;
		for (int i = 0; i < path0.size() - 1; ++i) {
			int n_strip = path0[i].dist(path0[i + 1]) / ds0;
			double ds1 = path0[i].dist(path0[i + 1]) / (double)n_strip;
			theta_point_t n = path0[i + 1] - path0[i];
			n /= n.norm();
			for (int j = 0; j < n_strip; ++j) {
				path.push_back(path0[i] + (double)j * ds1 * n);
			}
		}
		path.push_back(path0.back());
		return;

		//pathÇÃíºê¸ïîï™ÇëùÇ‚Ç∑
		int i_cp = 0;
		int i_next = 2;
		path.clear();
		path.push_back(path0[0]);
		while (1) {
			if (check_line(path0[i_cp], path0[i_next], ds)) {
				++i_next;
				if (i_next == path0.size()) {
					path.push_back(path0.back());
					return;
				}
			}
			else {
				path.push_back(path0[i_next - 1]);
				i_cp = i_next;
				++i_next;
			}
		}
	}
};