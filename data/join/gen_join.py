import numpy as np

inner_size = 10     # within data type range. uint64 : under 2^64
outer_size = 10

k1 = np.random.permutation(inner_size).astype(np.uint64)                    # 0 ~ n-1 without overlap
v1 = np.random.randint(0, 64, size=inner_size, dtype=np.uint64)             # 0 ~ 63
inner_rel = np.array([[k, v] for k, v in zip(k1, v1)], dtype=np.uint64)
np.save("inner", inner_rel)

print(f"inner_rel : (shape: {inner_rel.shape}):")
print(inner_rel)

k2 = np.random.randint(0, inner_size, size=outer_size, dtype=np.uint64)     # 0 ~ n-1 with overlap
v2 = np.random.randint(0, 64, size=outer_size, dtype=np.uint64)             # 0 ~ 63
outer_rel = np.array([[k, v] for k, v in zip(k2, v2)], dtype=np.uint64)
np.save("outer", outer_rel)

print(f"outer_rel : (shape: {outer_rel.shape}):")
print(outer_rel)

v2_sum = np.sum(v2)
np.save("sum", v2_sum)
print("sum", v2_sum)