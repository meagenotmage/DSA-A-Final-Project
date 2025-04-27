### Project: Task Management System with Priority Scheduling

#### Key Features:
# 1. Task Class:
#    - Each task will have attributes like a task ID, description, priority, and deadline.
#    - Tasks can have dependencies (other tasks that must be completed before this task).

# 2. Priority Queue:
#    - Use a heap or binary heap to store tasks, where the priority of a task determines its order in the queue.
#    - The priority queue will enable fast retrieval of the highest-priority task to work on next.

# 3. Graph Data Structure:
#    - Use a graph to model the dependencies between tasks.
#    - You can implement a topological sort to ensure tasks are executed in the correct order based on dependencies.

# 4. Algorithms:
#    - Depth-First Search (DFS) or Breadth-First Search (BFS) for exploring task dependencies.
#    - Dijkstra’s algorithm for scheduling tasks based on deadlines (or other weight-based scheduling criteria).
#    - Greedy algorithm for scheduling based on priority.
   
# 5. User Interface:
#    - Build a simple command-line interface where users can:
#      - Add new tasks with priority, deadline, and dependencies.
#      - View the task list sorted by priority or deadline.
#      - Mark tasks as complete and remove them from the queue.
#      - View task dependencies.

#### Implementation Details:
# - Data structures: Use Python's `heapq` for the priority queue, dictionaries and lists for managing the graph of task dependencies.
# - Algorithms: Implement sorting and searching algorithms as well as topological sorting for task execution.



### Algorithms to be used
# 1. Task Scheduling Based on Priority:
# Use a Min-Heap or Priority Queue (Python's heapq):
# Tasks can be prioritized by their priority level or deadline.
# Min-heap provides efficient access to the highest-priority (or earliest deadline) task with O(log n) insertion and retrieval time.
# Algorithm: Insert tasks into the heap based on their priority or deadline and always retrieve the task with the highest priority (smallest number) first.
# Why?: A heap ensures efficient retrieval of the next task to execute, making it ideal for priority-based scheduling. This is much faster than scanning a list for the next task.

# 2. Handling Task Dependencies:
# Use Topological Sorting via DFS (or Kahn’s Algorithm with BFS):
# When tasks have dependencies (e.g., Task A must be completed before Task B), you need to determine the correct order of execution.
# DFS-based topological sort is useful for generating a valid task order by exploring the dependencies. It detects cycles and ensures tasks are processed only when all dependent tasks are complete.
# Kahn’s Algorithm (BFS-based) is another option for topological sorting. It avoids recursion and processes tasks level-by-level based on their incoming dependencies.
# Why?: Topological sorting ensures tasks are executed in the correct order, respecting dependencies. If there’s no valid order (due to cycles), the system can alert the user.

# 3. Conflict-Free Scheduling Based on Deadlines or Dependencies:
# Dijkstra’s Algorithm (if tasks have deadlines or weights):
# If you need to prioritize tasks based on shortest paths (e.g., minimizing time to complete all tasks or considering deadlines), Dijkstra’s algorithm works well for finding the optimal path with weighted edges.
# You can assign weights (such as time until due or dependency complexity) and use Dijkstra’s algorithm to minimize the overall time to complete tasks.
# Why?: If you're looking to optimize task scheduling based on both priority and the time required to complete tasks, Dijkstra’s algorithm helps ensure that deadlines are respected and tasks are completed in an optimal order.

# 4. Detecting Cycles in Dependencies:
# DFS for Cycle Detection:
# When modeling dependencies between tasks, you must ensure there are no cycles in the task graph (a situation where Task A depends on Task B, but Task B also depends on Task A, directly or indirectly).
# Use DFS to detect cycles and prevent infinite loops in task scheduling.
# Why?: Cycle detection is crucial when dealing with dependencies. If cycles exist, it means the tasks cannot be executed in a valid order, and you need to alert the user to fix the dependencies.

# Summary of Algorithms to Use:
# Priority Queue (Heap) for scheduling tasks based on priority or deadline.
# Topological Sorting (DFS or Kahn’s Algorithm) for handling dependencies and determining a valid task order.
# Dijkstra’s Algorithm if you’re optimizing task completion based on weights like time or deadlines.
# DFS for Cycle Detection to ensure there are no invalid dependencies between tasks.

# Logic for Automatic Priority Assignment:
# High Priority (1): Task is urgent (e.g., deadline is within 2 days).
# Medium Priority (2): Task is important, but the deadline is more than 2 days away.
# Low Priority (3): Task is general, non-urgent.