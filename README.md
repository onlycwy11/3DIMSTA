# M2TAP: A 3D Indoor Multilayer Spatial Task Assignment Platform

This repository contains supplementary materials for the demo paper **"M2TAP: A 3D Indoor Multilayer Spatial Task Assignment Platform"**.

## 📝 Note on Source Code Availability

Due to company intellectual property and confidentiality requirements, we are unable to publicly disclose the full algorithmic source code. This repository only includes the **approximation ratio proof** of our proposed conflict-resolution approximation algorithm.

---

## 💡 Approximation Ratio Proof for the Conflict-Resolution Algorithm

### 1. Problem Definition
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
The task completion time that a robot $r$ executes a task $t$ is defined as $\tau(r, t)$.

*Definition 7 (3DIMSTA Problem):*
Given a map $M$, a set of robots $R$ and a set of micro-tasks $T$ on a spatial crowdsourcing platform, which has no task initially and allows that tasks can arrive in batches within a fixed time window, the **3DIMSTA** problem is to find an assignment schema $A \subseteq R \times T$ among robots and tasks to minimize the total completion time
$$MinSum(A) = \min \sum_{(r, t) \in A} \tau (r, t)$$
of all tasks in the batch, such that the following constraints are satisfied:
- **Task Arrival Constraint**: Upon posting, a task is either assigned to a compatible-type robot immediately, or deferred to the next batch window.
- **One-to-One Matching Constraint**: Tasks and robots are matched 1 to 1, until all tasks are assigned or all available robots are busy.
- **Invariability Constraint**: The assignment pair $(r,t)$ is fixed once task $t$ is assigned to robot $r$.
- **Path Conflict Constraint**: Robot paths are conflict-free, meaning that elevator contention should be resolved by selecting users, with others waiting or rerouting.

### 2. Approximation Ratio Proof

#### 2.1 Algorithm Recap

Our proposed **greedy conflict-resolution algorithm** follows the core principle of **minimizing path replacement cost** (incremental time cost), designed to resolve spatio-temporal elevator conflicts in 3D indoor multilayer scenarios. The algorithm flow is as follows:

**Input**: Initial task assignment $A_{ini}$.
1.  **Conflict Detection**: Check the initial assignment $A_{ini}$, traverse all time windows, and locate all sets of robots with elevator conflicts and their corresponding time windows $C$.
2.  **Greedy Resolution**: Let $A \leftarrow A_{ini}$. If $C = \emptyset$, proceed to step (4); otherwise, for each conflict $c_k \in C$:
    - Traverse the time window corresponding to conflict $c_k$, and obtain the set of conflicting robots $R_k$ in this window.
    - For each robot $r_i \in R_k$, calculate the **time cost increment $\Delta \tau_i$** when updating its original assignment $A_{i_{old}}$ to the new assignment $A_{i_{new}}$ (including the original task, new execution route, new time consumption, etc.) to avoid conflicts (waiting for elevator availability or switching to a suboptimal path).
    - Select the robot $r_s$ with the **maximum cost increment** (i.e., $\Delta \tau_s = \max_{r_i \in R_k} \{\Delta \tau_i\}$) and retain its original assignment; update the assignments of the remaining robots: $A \leftarrow A - A_{i_{old}}, A \leftarrow A \cup A_{i_{new}}$.
3.  **Iterative Loop**: Return to step (1), repeat conflict detection and resolution until no conflicts exist.
4.  **Output Result**: Let $A_{final} \leftarrow A$, output the final conflict-free task assignment scheme.

#### 2.2 Basic Definitions and Assumptions

**2.2.1 Standard Notation**

To avoid symbol conflicts, the following parameters are uniformly defined:

- $\text{APT}$: Total completion time of the algorithm’s output schedule
- $\text{OPT}$: Total completion time of the optimal conflict-free schedule
- $S_{\text{ini}}$: Total completion time of the initial algorithm assignment (without considering elevator conflicts)
- $\tau_{i_{\text{old}}}$: Original task completion time of robot $r_i$ before conflict resolution
- $\tau_{i_{\text{new}}}$: Updated task completion time of robot $r_i$ after conflict resolution
- $\Delta \tau_i$: Time cost increment for robot $r_i$, defined as $\Delta \tau_i = \tau_{i_{\text{new}}} - \tau_{i_{\text{old}}}$
- $\tau_{i_{\text{elevator}}}$: Task completion time of robot $r_i$ via elevator path
- $\tau_{i_{\text{stairs}}}$: Task completion time of robot $r_i$ via staircase path
- $d$: Horizontal distance between elevator entrance and staircase entrance
- $D$: Horizontal travel time between elevator entrance and staircase entrance
- $x_i$: Horizontal travel time from robot $r_i$ to the elevator entrance
- $y_i$: Horizontal travel time from robot $r_i$ to the staircase entrance
- $\alpha_i$: Vertical travel time of robot $r_i$ via staircase
- $\beta_i$: Vertical travel time of robot $r_i$ via elevator
- $n_i$: Number of floors traversed by robot $r_i$
- $m$: Time ratio between staircase and elevator

**2.2.2 Key Premises**

1. The initial assignment $A_{\text{ini}}$, obtained using Dijkstra's algorithm and the KM algorithm, is optimal without elevator conflicts, so:
   $$S_{\text{ini}} \le \text{OPT}$$
2. In the worst case after conflict resolution:
   $$\tau_{i_{\text{elevator}}} \le \tau_{i_{\text{old}}} \le \tau_{i_{\text{new}}} \le \tau_{i_{\text{stairs}}}$$
3. Time cost increment is bounded by:
   $$\Delta \tau_i \le \tau_{i_{\text{stairs}}} - \tau_{i_{\text{elevator}}}$$
4. Time ratio between staircase and elevator:
   - Horizontal speed: 1.5 $\text{m/s}$
   - Stair speed: 0.5 $\text{m/s}$
   - Floor height: 3.5 $\text{m}$
   - Stair time per floor: 7 $\text{s}$
   - Elevator time: 
     - open/close door: 1.5 $s$
     - runtime: 1.75 $\text{s/floor}$
     - move $n_i$ layers: 4.5 + 1.75$n_i$
   
   Thus:
   
$$\alpha_i = 7n_i$$
  
$$\beta_i = 4.5 + 1.75n_i$$

$$D = \frac{d}{1.5}$$

$$m = \frac{7n_i}{4.5 + 1.75n_i} = \frac{28n_i}{18 + 7n_i} < 4$$
   
5. According to fire safety and evacuation regulations [1][2], the horizontal distance between elevator and staircase satisfies $d < 25$.

#### 2.3 Approximation Ratio Analysis

The approximation ratio $\sigma$ is defined as the ratio of the total completion time of our algorithm (APT) to the optimal total time (OPT):

$$\sigma = \frac{\text{APT}}{\text{OPT}} \le \frac{\text{APT}}{S_{\text{ini}}}$$

- Total time before conflict resolution:
  $$S_{\text{ini}} = \sum \tau_{i_{\text{old}}}(r_i, t_i)$$
- Total time after conflict resolution:
  $$\text{APT} = \sum \tau_{i_{\text{new}}}(r_i, t_i)$$

From the worst-case path assumption:

$$\frac{\text{APT}}{A_{\text{ini}}} = \frac{\sum \tau_{i_{\text{new}}}}{\sum \tau_{i_{\text{old}}}} \le \frac{\sum \tau_{i_{\text{stairs}}}}{\sum \tau_{i_{\text{elevator}}}} $$

Let $\sum \tau_{i_{\text{stairs}}} = \sum a_i$ and $\sum \tau_{i_{\text{elevator}}} = \sum b_i$.

We only need to prove:

$$\frac{\sum a_i}{\sum b_i} < 4$$

**2.3.1 Path Decomposition and Three Scenarios**
Each robot’s travel time consists of **horizontal segment** and **vertical segment**.

- Staircase path: $a_i = y_i + \alpha_i$
- Elevator path: $b_i = x_i + \beta_i$

We analyze three relative position scenarios:

_Scenario 1: Elevator → Staircase → Robot (same side)_

$$y_i = x_i - D$$

$$\frac{\sum a_i}{\sum b_i} = \frac{\sum (x_i - D + \alpha_i)}{\sum (x_i + \beta_i)} \le \frac{\sum (-D + \alpha_i)}{\sum \beta_i}$$

_Scenario 2: Staircase → Elevator → Robot (same side)_

$$y_i = x_i + D$$

$$\frac{\sum a_i}{\sum b_i} = \frac{\sum (x_i + D + \alpha_i)}{\sum (x_i + \beta_i)} \le \frac{\sum (D + \alpha_i)}{\sum \beta_i}$$

_Scenario 3: Elevator and Staircase on opposite sides of Robot_

$$y_i = D - x_i$$

$$\frac{\sum a_i}{\sum b_i} = \frac{\sum (D - x_i + \alpha_i)}{\sum (x_i + \beta_i)} \le \frac{\sum (D + \alpha_i)}{\sum \beta_i}$$

**2.3.2 Auxiliary Theorem**

**Theorem 1**:
If for $\forall i \in \{1, 2, \dots, n\}$, $m_1 < \frac{a_i}{b_i} < m_2$ and $a_i, b_i > 0$, then:

$$m_1 < \frac{\sum a_i}{\sum b_i} < m_2$$

**Proof**:

$$ m_1 b_i < a_i < m_2 b_i $$

$$ \sum m_1 b_i < \sum a_i < \sum m_2 b_i $$

$$ m_1 \sum b_i < \sum a_i < m_2 \sum b_i $$

$$ m_1 < \frac{\sum a_i}{\sum b_i} < m_2 $$

**2.3.3 Upper Bound Verification**

Substitute $\alpha_i = 7n_i$ and $\beta_i = 4.5 + 1.75n_i$.

**For Scenario 1**:

$$\frac{-4D + 28n_i}{18 + 7n_i} = 4 - \frac{4D + 72}{18 + 7n_i}$$

Since $d > 0$, we have $D > 0$:

$$\frac{-4D + 28n_i}{18 + 7n_i} < 4$$

By Theorem 1:

$$\frac{\sum (-D + \alpha_i)}{\sum \beta_i} < 4$$

**For Scenarios 2 and 3:**

$$\frac{4D + 28n_i}{18 + 7n_i} = 4 + \frac{4D - 72}{18 + 7n_i}$$

Since $d \le 25$, we have $D \le \frac{50}{3}$:

$$\frac{4D + 28n_i}{18 + 7n_i} < 4$$

By Theorem 1:

$$\frac{\sum (D + \alpha_i)}{\sum \beta_i} < 4$$

In all three scenarios:

$$\frac{\sum a_i}{\sum b_i} < 4$$

Thus:

$$\sigma = \frac{\text{APT}}{\text{OPT}} \le \frac{\text{APT}}{S_{\text{ini}}} \le \frac{\sum a_i}{\sum b_i} < 4$$

The approximation ratio of our greedy conflict-resolution algorithm is **strictly less than 4**.

---

## References
<a id="ref-1">[1] China Academy of Building Research. Code for Design on Fire Protection of Building (GB 50016-2014, 2018 Edition). 2025. https://gf.cabr-fire.com/article-27251.htm

<a id="ref-1">[2] Technical Points for Fire Safety Evacuation Passages and Exits in Crowded Places. Shanghai Emergency Management Bureau, 2012. https://yjglj.sh.gov.cn/xxgk/xxgkml/zcfg/aqbz/20120312/0037-21550.html
