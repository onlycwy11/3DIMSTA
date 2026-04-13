# M2TAP: A 3D Indoor Multilayer Spatial Task Assignment Platform

This repository contains supplementary materials for the demo paper **"M2TAP: A 3D Indoor Multilayer Spatial Task Assignment Platform"**.

## 📝 Note on Source Code Availability

Due to company intellectual property and confidentiality requirements, we are unable to publicly disclose the full algorithmic source code. This repository only includes the **approximation ratio proof** of our proposed conflict-resolution approximation algorithm.

---

## 💡 Approximation Ratio Proof for the Conflict-Resolution Algorithm

### 1.Problem Definition
We consider the **3D indoor multilayer spatial task assignment (3DIMSTA)** problem, defined as follows:

*Definition 1 (Node):*
A node, denoted by $n = \langle f_{n}, type_{n} \rangle$, is located on floor $f_n$ and possesses a type $type_n$ (e.g., planar or vertical).

*Definition 2 (Edge):*
An edge, denoted by $a = \langle l_{a}, type_{a} \rangle$, is a connection between two nodes with length $l_a$. The type $type_a$ specifies which entities are allowed to traverse this edge.

*Definition 3 (Map Model):*
A 3D topological map ("map" for short), denoted by $M = \langle V, E \rangle$ is defined by a node set $V$ and an edge set $E$.

*Definition 4 (Robot):*
A robot, denoted by $r = \langle type_r, p_r \rangle$, is a task executor with specific type $type_r$ (e.g., wheeled or legged) deployed in a 3D space. Its initial spatial node on the platform is $p_r$.

*Definition 5 (Micro-Task):*
A two-stage micro-task ("task" for short), denoted by $t = \langle type_t, s_t, g_t \rangle$, arrives at the platform within a fixed time window and either requires a robot with type $type_t$ to execute it from the pickup node $s_t$ to the delivery node $g_t$, or remains unassigned until a subsequent scheduling opportunity.

*Definition 6 (Task Completion Time):*
The task completion time that a robot $r$ executes a task $t$ is defined as $C(r, t)$.

*Definition 7 (3DIMSTA Problem):*
Given a map $M$, a set of robots $R$ and a set of micro-tasks $T$ on a spatial crowdsourcing platform, which has no task initially and allows that tasks can arrive in batches within a fixed time window, the **3DIMSTA** problem is to find an assignment schema $A \subseteq R \times T$ among robots and tasks to minimize the total completion time
$$MinSum(A) = \min \sum_{(r, t) \in A} C(r, t)$$
of all tasks in the batch, such that the following constraints are satisfied:

- **Task Arrival Constraint**: Upon posting, a task is either assigned to a compatible-type robot immediately, or deferred to the next batch window.
- **One-to-One Matching Constraint**: Tasks and robots are matched 1 to 1, until all tasks are assigned or all available robots are busy.
- **Invariability Constraint**: The assignment pair $(r,t)$ is fixed once task $t$ is assigned to robot $r$.
- **Path Conflict Constraint**: Robot paths are conflict-free, meaning that elevator contention should be resolved by selecting users, with others waiting or rerouting.
