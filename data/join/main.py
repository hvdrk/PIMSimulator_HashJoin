import numpy as np

# (1) 8바이트 정수 타입으로 변경
tuple_dtype_64 = np.dtype([('key', np.int64), ('value', np.int64)])

def generate_join_data(num_tuples):
    rng = np.random.default_rng()

    # (2) R 테이블: 8바이트 정수로 생성
    R_keys = rng.integers(0, num_tuples // 10, size=num_tuples, dtype=np.int64)
    R_values = np.arange(num_tuples, dtype=np.int64)
    R = np.array(list(zip(R_keys, R_values)), dtype=tuple_dtype_64)

    # (3) S 테이블: 8바이트 정수로 생성
    #     S_keys는 R_keys에서 임의 추출
    #     S_values는 num_tuples~(2*num_tuples-1)
    S_keys = rng.choice(R_keys, size=num_tuples, replace=True)
    S_values = np.arange(num_tuples, num_tuples * 2, dtype=np.int64)
    S = np.array(list(zip(S_keys, S_values)), dtype=tuple_dtype_64)

    # (4) Join 결과: key, r_value, s_value 모두 8바이트
    result_list = []
    for r in R:
        for s in S:
            if r['key'] == s['key']:
                result_list.append((r['key'], r['value'], s['value']))
                break

    # (5) result_list를 최종 배열로 변환
    result = np.array(
        result_list,
        dtype=[('key', np.int64), ('r_value', np.int64), ('s_value', np.int64)]
    )

    # (6) 저장
    np.save("R.npy", R)
    np.save("S.npy", S)
    np.save("result.npy", result)

    print(f"Generated {num_tuples} tuples for R and S, {len(result)} join results")

    print("R is :", R)


if __name__ == "__main__":
    num_tuples = 128  #128M
    generate_join_data(num_tuples)