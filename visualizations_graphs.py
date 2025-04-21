import matplotlib.pyplot as plt
import time

MAX_RUNTIME = 10  # seconds
NUM_ROCKETS = 1

def generate_objects(num_cargo, num_locations):
    rockets = [f"r{i}" for i in range(NUM_ROCKETS)]
    cargo = [f"c{i}" for i in range(num_cargo)]
    locs = [f"loc{i}" for i in range(num_locations)]
    return rockets, cargo, locs

def generate_ground_ops(rockets, cargo, locs):
    ops = []
    for r in rockets:
        for l1 in locs:
            for l2 in locs:
                if l1 != l2:
                    ops.append({
                        "name": "move",
                        "ground": {"?r": r, "?from": l1, "?to": l2},
                        "pre": [f"at({r},{l1})"],
                        "add": [f"at({r},{l2})"],
                        "del": [f"at({r},{l1})"],
                    })
        for c in cargo:
            for l in locs:
                ops.append({
                    "name": "load",
                    "ground": {"?c": c, "?r": r, "?l": l},
                    "pre": [f"at({c},{l})", f"at({r},{l})", f"free({r})"],
                    "add": [f"in({c},{r})"],
                    "del": [f"at({c},{l})", f"free({r})"],
                })
                ops.append({
                    "name": "unload",
                    "ground": {"?c": c, "?r": r, "?l": l},
                    "pre": [f"in({c},{r})", f"at({r},{l})"],
                    "add": [f"at({c},{l})", f"free({r})"],
                    "del": [f"in({c},{r})"],
                })
    return ops

def apply(state, action):
    return list({*(p for p in state if p not in action["del"]), *action["add"]})

def key(state):
    return tuple(sorted(state))

def search(strategy, ops, init_state, goal_state, timeout):
    frontier = [init_state]
    parent, action_from = {key(init_state): None}, {}
    nodes = 0
    start = time.time()
    while frontier:
        if time.time() - start > timeout:
            return None, None, None, nodes, False
        cur = frontier.pop() if strategy == "dfs" else frontier.pop(0)
        cur_k = key(cur)
        nodes += 1
        if all(g in cur for g in goal_state):
            return parent, action_from, cur_k, nodes, True
        for a in ops:
            if all(p in cur for p in a["pre"]):
                nxt = apply(cur, a)
                k = key(nxt)
                if k not in parent:
                    parent[k] = cur_k
                    action_from[k] = a
                    frontier.append(nxt)
    return {}, {}, None, nodes, True

def run_graphplan(ops, init_state, goal_state, timeout, max_levels=15):
    S_levels = [set(init_state)]
    A_levels = []
    start = time.time()
    for lvl in range(max_levels):
        if time.time() - start > timeout:
            return S_levels, A_levels, False, time.time() - start
        applicable = [a for a in ops if all(p in S_levels[-1] for p in a["pre"])]
        A_levels.append(applicable)
        next_S = set(S_levels[-1])
        for a in applicable:
            next_S.update(a["add"])
        if all(g in next_S for g in goal_state):
            return S_levels[:lvl+2], A_levels[:lvl+1], True, time.time() - start
        if next_S == S_levels[-1]:
            break
        S_levels.append(next_S)
    return S_levels, A_levels, False, time.time() - start

def benchmark_varying_cargo(max_cargo=12):
    graphplan_times, bfs_times, dfs_times = [], [], []
    for cargo in range(1, max_cargo + 1):
        rockets, cargo_objs, locs = generate_objects(cargo, 4)
        ops = generate_ground_ops(rockets, cargo_objs, locs)
        init_state = [f"at({r},{locs[0]})" for r in rockets] + [f"at({c},{locs[0]})" for c in cargo_objs] + [f"free({r})" for r in rockets]
        goal_state = [f"at({c},{locs[-1]})" for c in cargo_objs]
        # GraphPlan
        _, _, _, t = run_graphplan(ops, init_state, goal_state, MAX_RUNTIME)
        graphplan_times.append(t * 1000)
        # BFS
        t0 = time.time()
        _, _, _, _, success = search("bfs", ops, init_state, goal_state, MAX_RUNTIME)
        bfs_times.append((time.time() - t0) * 1000 if success else None)
        # DFS
        t0 = time.time()
        _, _, _, _, success = search("dfs", ops, init_state, goal_state, MAX_RUNTIME)
        dfs_times.append((time.time() - t0) * 1000 if success else None)
    return graphplan_times, bfs_times, dfs_times

def benchmark_varying_locations(max_locs=12):
    graphplan_times, bfs_times, dfs_times = [], [], []
    for locs in range(2, max_locs + 1):
        rockets, cargo_objs, locs_list = generate_objects(3, locs)
        ops = generate_ground_ops(rockets, cargo_objs, locs_list)
        init_state = [f"at({r},{locs_list[0]})" for r in rockets] + [f"at({c},{locs_list[0]})" for c in cargo_objs] + [f"free({r})" for r in rockets]
        goal_state = [f"at({c},{locs_list[-1]})" for c in cargo_objs]
        # GraphPlan
        _, _, _, t = run_graphplan(ops, init_state, goal_state, MAX_RUNTIME)
        graphplan_times.append(t * 1000)
        # BFS
        t0 = time.time()
        _, _, _, _, success = search("bfs", ops, init_state, goal_state, MAX_RUNTIME)
        bfs_times.append((time.time() - t0) * 1000 if success else None)
        # DFS
        t0 = time.time()
        _, _, _, _, success = search("dfs", ops, init_state, goal_state, MAX_RUNTIME)
        dfs_times.append((time.time() - t0) * 1000 if success else None)
    return graphplan_times, bfs_times, dfs_times

# Run benchmarks
cargo_results = benchmark_varying_cargo(12)
loc_results = benchmark_varying_locations(12)

# Plotting
def plot_results(x, graphplan, bfs, dfs, title, xlabel):
    plt.figure(figsize=(10, 6))
    plt.plot(x, graphplan, label="GraphPlan", marker="o")
    plt.plot(x, bfs, label="BFS", marker="o")
    plt.plot(x, dfs, label="DFS", marker="o")
    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel("Runtime (ms)")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()

plot_results(range(1, 13), *cargo_results, "Runtime vs Number of Cargo", "Number of Cargo")
plot_results(range(2, 13), *loc_results, "Runtime vs Number of Locations", "Number of Locations")
