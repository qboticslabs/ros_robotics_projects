import tensorflow as tf

sess = tf.Session()

matrix1 = tf.constant([[3., 3.],[4., 4.]])
matrix2 = tf.constant([[3., 3.],[4., 4.]])

message = tf.constant('Results of matrix operations')

product = tf.matmul(matrix1, matrix2)
det = tf.matrix_determinant(matrix1)

result1 = sess.run(product)
result2 = sess.run(det)

print(sess.run(message))
print(result1)
print("\n")
print(result2)

sess.close()
