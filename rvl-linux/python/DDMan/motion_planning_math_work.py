import numpy as np
import rvlpyutil as rvl
import matplotlib.pyplot as plt

def test1():
	A = np.random.rand(2,6)

	pivot_idx = np.zeros(2).astype('int')
	A2 = np.zeros((2,8))
	A2[:,:6] = A
	A2[:,6:] = -np.eye(2)
	pivot_idx[0] = np.argmax(np.abs(A2[0,:6]))
	A2[0,:] /= A2[0,pivot_idx[0]]
	A2[1,:] -= A2[1,pivot_idx[0]] * A2[0,:]
	pivot_idx[1] = np.argmax(np.abs(A2[1,:6]))
	A2[1,:] /= A2[1,pivot_idx[1]]
	A2[0,:] -= A2[0,pivot_idx[1]] * A2[1,:]

	other_idx = np.setdiff1d(np.arange(6),pivot_idx)
	A3 = A2[:,other_idx]
	A4 = A2[:,6:]

	for i in range(30):
		x2=2 * np.random.rand(4,1) - 1
		s=np.random.rand(2,1)
		x1=-A3@x2-A4@s
		x=np.zeros((6,1))
		x[pivot_idx,:] = x1
		x[other_idx,:] = x2
		print(A@x)
		
def demo_6d():
	n = 6
	m = 10
	load_from_file = False
	verbose = False

	for iTest in range(10):
		print('Test %d' % iTest)
		print(' ')
		if load_from_file:
			data = np.load('data.npy')
			m = data.shape[0]
			n = data.shape[1]-1
			A = data[:,:n]
			b = data[:,n,np.newaxis]
		else:
			A = 2.0 * np.random.rand(m, n) - 1.0
			b = 2.0 * np.random.rand(m, 1) - 1.0
			np.save('data', np.concatenate((A,b), 1))
		
		x0 = np.zeros((n,1))
				
		x, feasible_solution_exists = rvl.feasible_solution(A, b, x0, verbose=verbose)
		
		if feasible_solution_exists:
			print('Feasible solution is:')
			print(x)
			maxe = np.max(A@x-b)
			print('max(e)=%f' % maxe)
			print(' ')
			if maxe > 1e-10:
				print('Error! False feasible solution!')
				break
		else:
			print('No feasible solution exists.')
		input("Press Enter to continue...")

def demo_2d():
	n = 2
	m = 10
	
	for iTest in range(10):
		print('Test %d' % iTest)
		print(' ')

		c = 2.0 * np.random.rand(m, n) - 1.0
		v = np.stack((-c[:,1], c[:,0]), 1)
		r = 2.0
		a_ = np.sum(v*v, 1)
		c_ = np.sum(c*c, 1) - r**2
		s = np.sqrt(- 4 * a_ * c_) / (2.0 * a_)
		x1 = c + s[:,np.newaxis] * v
		x2 = c - s[:,np.newaxis] * v
		b = np.linalg.norm(c,axis=1)[:,np.newaxis]
		A = c/b
		x0 = 2.0 * np.random.rand(n,1) - 1.0
		x, feasible_solution_exists = rvl.feasible_solution(A,b,x0)
		print(x)
		if feasible_solution_exists:
			c_nrm = c + 0.1*A
			plt.plot(np.stack((x1[:,0], x2[:,0]),0), np.stack((x1[:,1], x2[:,1]),0))
			plt.plot(np.stack((c[:,0], c_nrm[:,0]),0), np.stack((c[:,1], c_nrm[:,1]),0))
			plt.plot(x0[0], x0[1], marker='+', markerfacecolor='red')
			plt.plot(x[0], x[1], marker='x', markerfacecolor='green')
			plt.grid()
			plt.show()
		else:
			print('No feasible solution exists.')
		
		
		
	
						
					
					
					
					
					
		
