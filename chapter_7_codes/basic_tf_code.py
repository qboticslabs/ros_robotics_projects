#!/usr/bin/env python

import tensorflow as tf
import time

matrix_1 = tf.Variable([[1,2,3],[4,5,6],[7,8,9]],name="mat1")
matrix_2 = tf.Variable([[1,2,3],[4,5,6],[7,8,9]],name="mat2")

scalar = tf.constant(5)
number = tf.Variable(1, name="counter")

add_msg = tf.constant("\nResult of matrix addition\n")
mul_msg = tf.constant("\nResult of matrix multiplication\n")
scalar_mul_msg = tf.constant("\nResult of scalar multiplication\n")
number_mul_msg = tf.constant("\nResult of Number multiplication\n")


mat_add = tf.add(matrix_1,matrix_2)
mat_mul = tf.matmul(matrix_1,matrix_2)
mat_scalar_mul = tf.mul(scalar,mat_mul)
mat_number_mul = tf.mul(number,mat_mul)


init_op = tf.initialize_all_variables()
sess = tf.Session()
tf.device("/cpu:0")
sess.run(init_op)

for i in range(1,100):

	print "\nFor i =",i

	print(sess.run(add_msg))
	print(sess.run(mat_add))

	print(sess.run(mul_msg))
	print(sess.run(mat_mul))

	print(sess.run(scalar_mul_msg))
	print(sess.run(mat_scalar_mul))

	update = tf.assign(number,tf.constant(i))
	sess.run(update)
	
	print(sess.run(number_mul_msg))
	print(sess.run(mat_number_mul))

	time.sleep(0.1)
	

sess.close()
