## Verifying `achieved_goal` vs `desired_goal`

### Why this check matters

In goal‑conditioned environments, it is easy to mix up:

* what the **robot is currently doing**, and
* what the **task is asking the robot to reach**.

Before relying on these values for rewards or training, I wanted to explicitly verify **what each goal represents** in this environment.

The key question was:

> Does `achieved_goal` represent the **object position**, or the **robot end‑effector (EE) position**?

---

### Coordinate frame intuition (plain language)

To reason about the numbers, I used the following mental model:

* **X axis**: starts at 0 on the left, increases to the right
* **Y axis**: starts near 0 at the top, becomes more negative as we move downward
* **Z axis**: starts at 0 on the ground and increases upward

From a **top‑down (plan) view**:

* Changing **Y** moves the robot forward/backward on the table
* **Z** should stay almost constant when pushing an object on the surface

---

### What I did (experiment setup)

1. Printed the **end‑effector position** and the **object position**.
2. Initially aligned the arm with the object in **x** and **z**, but kept it far away in **y**.
3. Extended the arm forward to **push the apple along the y direction**.
4. Checked how both positions changed before and after contact.

---

### Before pushing the object

```
print(env._get_ee_pos())
print(env._get_obj_pos())
```

EE:

```
[ 0.703 -0.789  0.961]
```

Object:

```
[ 0.702 -0.598  0.952]
```

Observation:

* EE and object align in **x** and **z**
* Large difference in **y**, meaning no contact yet

---

### After pushing the object (along Y)

```
print(env._get_ee_pos())
print(env._get_obj_pos())
```

EE:

```
[ 0.708 -0.569  0.963]
```

Object:

```
[ 0.704 -0.535  0.955]
```

Observation:

* Both EE and object move in the **y direction**
* **Z remains nearly unchanged**, matching a surface push
* Object motion follows the EE after contact

---

### Interpretation

This controlled push confirms:

* `env._get_ee_pos()` corresponds to the **robot end‑effector position**
* `env._get_obj_pos()` corresponds to the **object position**

Mapping to goal terms:

* **`achieved_goal` → EE position**
* **`desired_goal` → object position**

---

### Conclusion

* Goal values are wired correctly in the environment
* EE movement updates `achieved_goal`
* Object motion updates `desired_goal`

This gives confidence when using these values for reward computation, debugging, and policy evaluation.
