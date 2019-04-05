#  ___________________________________________________________________________
#
#  Pyomo: Python Optimization Modeling Objects
#  Copyright 2017 National Technology and Engineering Solutions of Sandia, LLC
#  Under the terms of Contract DE-NA0003525 with National Technology and
#  Engineering Solutions of Sandia, LLC, the U.S. Government retains certain
#  rights in this software.
#  This software is distributed under the 3-clause BSD License.
#  ___________________________________________________________________________
from pyomo.contrib.pynumero.linalg.solvers.regularization import InertiaCorrectionParams
try:
    from pyomo.contrib.pynumero.linalg.solvers.ma27_solver import MA27LinearSolver
    found_ma27 = True
except ImportError as e:
    found_ma27 = False

try:
    from pyomo.contrib.pynumero.linalg.solvers.ma57_solver import MA57LinearSolver
    found_ma57 = True
except ImportError as e:
    found_ma57 = False

try:
    from pyomo.contrib.pynumero.linalg.solvers.mumps_solver import MUMPSSymLinearSolver
    found_mumps = True
except ImportError as e:
    found_mumps = False

import numpy as np
from pyomo.contrib.pynumero.sparse import (BlockSymMatrix,
                                           BlockVector,
                                           BlockMatrix,
                                           empty_matrix)

from scipy.sparse.linalg.interface import LinearOperator
from scipy.sparse.linalg import spsolve, inv, splu, cg
from scipy.sparse import coo_matrix, identity
import inspect
import six
import abc


@six.add_metaclass(abc.ABCMeta)
class KKTSolver(object):

    def __init__(self, linear_solver, **kwargs):
        self._lsolver = linear_solver

    @abc.abstractmethod
    def solve(self, kkt_matrix, rhs, nlp, *args, **kwargs):
        return

    @abc.abstractmethod
    def reset_inertia_parameters(self):
        return

    def __str__(self):
        return 'KKTSolver'

    def __repr__(self):
        return 'KKTSolver'


class FullKKTSolver(KKTSolver):

    def __init__(self, linear_solver, pivotol=1e-8, options=None):

        if options is None:
            options = dict()

        # create linear solver
        if linear_solver == 'mumps':
            if found_mumps:
                lsolver = MUMPSSymLinearSolver(pivotol, **options)
            else:
                if found_ma27:
                    print('WARNING: Running with ma27 linear solver. Mumps not available')
                    lsolver = MA27LinearSolver(pivotol, **options)
                raise RuntimeError('Did not found MUMPS linear solver')
        elif linear_solver == 'ma27':
            if found_ma27:
                lsolver = MA27LinearSolver(pivotol, **options)
            else:
                if found_mumps:
                    print('WARNING: Running with mumps linear solver. MA27 not available')
                    lsolver = MUMPSSymLinearSolver(pivotol, **options)
                raise RuntimeError('Did not found MA27 linear solver')
        elif linear_solver == 'ma57':
            if found_ma57:
                lsolver = MA57LinearSolver(pivotol, **options)
            else:
                if found_mumps:
                    print('WARNING: Running with mumps linear solver. MA57 not available')
                    lsolver = MUMPSSymLinearSolver(pivotol, **options)
                raise RuntimeError('Did not found MA57 linear solver')
        else:
            raise RuntimeError('{} Linear solver not recognized'.format(linear_solver))

        # call parent class to set model
        super(FullKKTSolver, self).__init__(lsolver)
        self._inertia_params = InertiaCorrectionParams()

    def solve(self, kkt, rhs, nlp, *args, **kwargs):
        do_symbolic = kwargs.pop('do_symbolic', True)
        max_iter_reg = kwargs.pop('max_iter_reg', 40)
        wr = kwargs.pop('regularize', True)
        max_iter_ref = kwargs.pop('max_iter_ref', 10)
        tol_iter_ref = kwargs.pop('tol_iter_ref', 1e-8)

        # set auxiliary variables
        diagonal = None
        num_eigvals = -1
        nvars = kkt.shape[0]
        lsolver = self._lsolver
        val_reg = 0.0
        count_iter = 0

        if wr:
            diagonal = np.zeros(kkt.shape[0])
            num_eigvals = nlp.nc + nlp.nd

        # do symbolic factorization
        if do_symbolic:
            lsolver.do_symbolic_factorization(kkt, include_diagonal=wr)

        # do numeric factorization
        status = lsolver.do_numeric_factorization(kkt,
                                                  diagonal=diagonal,
                                                  desired_num_neg_eval=num_eigvals)

        # regularize problem if required
        if wr:

            done = self._inertia_params.ibr1(status)
            nx = nvars - num_eigvals

            while not done and count_iter < max_iter_reg:

                diagonal[0: nx] = self._inertia_params.delta_w
                diagonal[nx: nvars] = self._inertia_params.delta_a
                status = self._lsolver.do_numeric_factorization(kkt,
                                                                diagonal=diagonal,
                                                                desired_num_neg_eval=num_eigvals)

                if self._inertia_params.delta_w > 0.0:
                    val_reg = self._inertia_params.delta_w
                done = self._inertia_params.ibr4(status)
                count_iter += 1

            if count_iter >= max_iter_reg:
                raise RuntimeError('Could not regularized the problem')

        info = {'status': status, 'delta_reg': val_reg, 'reg_iter': count_iter}

        # iterative refinement
        if max_iter_ref > 0:
            x = lsolver.do_back_solve(rhs)
            xr = x.flatten()
            flat_matrix = kkt.tocsr()
            flat_rhs = rhs.flatten()

            for i in range(max_iter_ref):
                res = flat_rhs - flat_matrix.dot(xr)
                d = lsolver.do_back_solve(res, flat_solution=True)
                xr += d
                #print(np.linalg.norm(res, ord=np.inf))
                if np.linalg.norm(res, ord=np.inf) <= tol_iter_ref:
                    break
            if isinstance(x, BlockVector):
                x.copyfrom(xr)
            else:
                x = xr
        else:
            x = lsolver.do_back_solve(rhs)

        return x, info

    def reset_inertia_parameters(self):
        self._inertia_params.reset()

    def __str__(self):
        if isinstance(self._lsolver, MUMPSSymLinearSolver):
            return 'mumps'
        return 'ma27'

    def __repr__(self):
        if isinstance(self._lsolver, MUMPSSymLinearSolver):
            return 'mumps'
        return 'ma27'


def build_permuted_kkt(kkt, nlp):

    # Note: this ignores Jd_coupling since they are empty blocks
    nblocks = nlp.nblocks

    permuted_kkt = BlockSymMatrix(nblocks+1)
    for bid in range(nblocks):

        hess = kkt[0, 0][bid, bid]
        Ds = kkt[1, 1][bid, bid]
        Jc = kkt[2, 0][bid, bid]
        Jd = kkt[3, 0][bid, bid]
        A = kkt[2, 0][bid + nblocks, bid]
        B = kkt[2, 0][bid + nblocks, nblocks]

        block_kkt = BlockSymMatrix(5)
        block_kkt[0, 0] = hess
        block_kkt[1, 1] = Ds
        block_kkt[2, 0] = Jc
        block_kkt[3, 0] = Jd
        nd = Ds.shape[0]
        if nd > 0:  # if is zero just keep that as none which is equivalent
            block_kkt[3, 1] = -identity(nd)
        block_kkt[4, 0] = A
        nzeros = hess.shape[0] + Ds.shape[0] + Jc.shape[0] + Jd.shape[0]
        nz = B.shape[0]
        block_B = BlockMatrix(1, 2)
        block_B[0, 0] = empty_matrix(nz, nzeros)
        block_B[0, 1] = B

        permuted_kkt[bid, bid] = block_kkt
        permuted_kkt[nblocks, bid] = block_B

    hess_z = kkt[0, 0][nblocks, nblocks]
    permuted_kkt[nblocks, nblocks] = hess_z
    return permuted_kkt


def build_permuted_rhs(rhs, nlp):

    # Note: this ignores Jd_coupling since they are empty blocks
    nblocks = nlp.nblocks
    permuted_rhs = BlockVector(nblocks + 1)
    for bid in range(nblocks):
        grad_x_lag = rhs[0][bid]
        grad_s_lag = rhs[1][bid]
        grad_yc_lag = rhs[2][bid]
        grad_yc_coupling_lag = rhs[2][bid + nblocks]
        grad_yd_lag = rhs[3][bid]

        block_rhs = BlockVector(5)
        block_rhs[0] = grad_x_lag
        block_rhs[1] = grad_s_lag
        block_rhs[2] = grad_yc_lag
        block_rhs[3] = grad_yd_lag
        block_rhs[4] = grad_yc_coupling_lag

        permuted_rhs[bid] = block_rhs

    grad_z_lag = rhs[0][nblocks]
    permuted_rhs[nblocks] = grad_z_lag

    return permuted_rhs


def build_schur_complement(kkt, nlp):
    # Note: just for testing
    nblocks = nlp.nblocks
    permuted_kkt = build_permuted_kkt(kkt, nlp)
    S = empty_matrix(nlp.nz, nlp.nz)
    for bid in range(nblocks):
        Ki = permuted_kkt[bid, bid].tocoo()
        BiT = permuted_kkt[bid, nblocks].tocoo()
        Bi = permuted_kkt[nblocks, bid].tocoo()
        Ri = spsolve(Ki, BiT)
        Si = Bi.dot(Ri)
        S -= Si
    return S


class TwoStageStochasticSchurKKTSolver(KKTSolver):

    def __init__(self, linear_solver):

        supported_solvers = ['ma27', 'ma57', 'mumps']
        if linear_solver not in supported_solvers:
            msg = 'Linear solver {} not supported'.format(linear_solver)
            raise RuntimeError(msg)

        if linear_solver == 'mumps':
            if found_mumps:
                lsolver = 'mumps'
            else:
                if found_ma27:
                    print('WARNING: Mumps not available. Running ma27.')
                    lsolver = 'mumps'
                raise RuntimeError('Did not found mumps linear solver')
        elif linear_solver == 'ma27':
            if found_ma27:
                lsolver = 'ma27'
            else:
                if found_mumps:
                    print('WARNING: Ma27 not found. Running mumps.')
                    lsolver = 'mumps'
                raise RuntimeError('Did not found ma27 linear solver')
        elif linear_solver == 'ma57':
            if found_ma57:
                lsolver = 'ma57'
            else:
                if found_mumps:
                    print('WARNING: Ma57 not found. Running mumps.')
                    lsolver = 'mumps'
                raise RuntimeError('Did not found ma57 linear solver')

        if lsolver == 'mumps':
            from mpi4py import MPI
            comm = MPI.COMM_WORLD
            assert comm.Get_size() == 1, 'Mumps only supported with one procesor'

        # call parent class to set model
        super(TwoStageStochasticSchurKKTSolver, self).__init__(linear_solver)
        self._lsolver = lsolver
        self._inertia_params = None
        self._diagonal = None

        # ToDo: this can probably be replaced with numpy solver
        if self._lsolver == 'mumps':
            self._sc_solver = MUMPSSymLinearSolver()
        elif self._lsolver == 'ma27':
            self._sc_solver = MA27LinearSolver()
        elif self._lsolver == 'ma57':
            self._sc_solver = MA57LinearSolver()

    def solve(self, kkt, rhs, nlp, **kwargs):

        do_symbolic = kwargs.pop('do_symbolic', True)
        max_iter_reg = kwargs.pop('max_iter_reg', 40)
        wr = kwargs.pop('regularize', True)

        # create auxiliary variables
        nblocks = nlp.nblocks
        val_reg_w = 0.0
        val_reg_a = 0.0
        max_count_iter = 0
        overall_status = 0
        permuted_kkt = build_permuted_kkt(kkt, nlp)
        nlps = [nlp_block for name, nlp_block in nlp.nlps()]
        status = [None for i in range(nblocks)]
        # run symbolic factorization if enabled
        if do_symbolic:
            # allocate linear solvers
            if self._lsolver == 'mumps':
                self._lsolver = [MUMPSSymLinearSolver() for i in range(nblocks)]
            elif self._lsolver == 'ma27':
                self._lsolver = [MA27LinearSolver() for i in range(nblocks)]
            elif self._lsolver == 'ma57':
                self._lsolver = [MA57LinearSolver() for i in range(nblocks)]
            # allocate diagonals and regularization helpers
            self._inertia_params = []
            diagonals = []
            for scenario in nlps:
                self._inertia_params.append(InertiaCorrectionParams())
                nx = scenario.nx
                nd = scenario.nd
                nc = scenario.nc
                diagonals.append(np.zeros(nx + 2 * nd + nc + nlp.nz))
            self._diagonal = diagonals

            # perform symbolic factorization in each block kkt
            for bid in range(nblocks):
                block_kkt = permuted_kkt[bid, bid]
                self._lsolver[bid].do_symbolic_factorization(block_kkt, include_diagonal=True)
        else:
            assert self._diagonal is not None, "No symbolic factorization has been done yet"

        # numeric factorization
        for bid, nlp_block in enumerate(nlps):
            block_kkt = permuted_kkt[bid, bid]
            num_eigvals = nlp_block.nc + nlp_block.nd + nlp.nz if wr else -1
            status[bid] = self._lsolver[bid].do_numeric_factorization(block_kkt,
                                                                      diagonal=self._diagonal[bid],
                                                                      desired_num_neg_eval=num_eigvals)

        # regularize if needed
        if wr:

            # first pass to see which block needs the largest regularization
            for bid, block_nlp in enumerate(nlps):
                block_kkt = permuted_kkt[bid, bid]
                done = self._inertia_params[bid].ibr1(status[bid])
                count_iter = 0
                nneval = block_nlp.nc + block_nlp.nd + nlp.nz
                nxd = block_nlp.nx + block_nlp.nd
                nvars = block_nlp.nx + 2 * block_nlp.nd + block_nlp.nc + nlp.nz

                while not done and count_iter < max_iter_reg:

                    self._diagonal[bid][0: nxd] = self._inertia_params[bid].delta_w
                    self._diagonal[bid][nxd: nvars] = self._inertia_params[bid].delta_a
                    status = self._lsolver[bid].do_numeric_factorization(block_kkt,
                                                                         diagonal=self._diagonal[bid],
                                                                         desired_num_neg_eval=nneval)

                    if val_reg_w > self._inertia_params[bid].delta_w:
                        val_reg_w = self._inertia_params[bid].delta_w

                    if val_reg_a > self._inertia_params[bid].delta_a:
                        val_reg_a = self._inertia_params[bid].delta_a

                    if count_iter > max_count_iter:
                        max_count_iter = count_iter

                    done = self._inertia_params[bid].ibr4(status)
                    count_iter += 1

            # regularize all blocks with same value
            if val_reg_a > 0.0 or val_reg_w > 0.0:

                for bid, block_nlp in enumerate(nlps):
                    block_kkt = permuted_kkt[bid, bid]
                    nneval = block_nlp.nc + block_nlp.nd + nlp.nz
                    nxd = block_nlp.nx + block_nlp.nd
                    nvars = block_nlp.nx + 2 * block_nlp.nd + block_nlp.nc + nlp.nz

                    self._diagonal[bid][0: nxd] = val_reg_w
                    self._diagonal[bid][nxd: nvars] = val_reg_a
                    status = self._lsolver[bid].do_numeric_factorization(block_kkt,
                                                                         diagonal=self._diagonal[bid],
                                                                         desired_num_neg_eval=nneval)

                    if status > overall_status:
                        overall_status = status

                    # reset diagonals to zero
                    self._diagonal[bid].fill(0.0)

        # permute rhs
        permuted_rhs = build_permuted_rhs(rhs, nlp)

        # build schur-complement and rhs-schur-complement
        sc_row = []
        sc_col = []
        sc_values = []
        rhs_sc = permuted_rhs[nblocks].copy()
        for bid in range(nblocks):
            lsolver = self._lsolver[bid]
            # compute schur-complement contribution of bid block
            BiT = permuted_kkt[bid, nblocks].tocoo()
            Bi = permuted_kkt[nblocks, bid].tocsr()
            nnz_cols = np.unique(BiT.col)
            BiT_csc = BiT.tocsc()
            for j in nnz_cols:
                si = BiT_csc.getcol(j).toarray()
                sij = lsolver.do_back_solve(si)
                sc_values.append(-Bi.dot(sij))
                sc_row.append(np.arange(nlp.nz))
                tmp = np.ones(nlp.nz, dtype='i')
                tmp.fill(j)
                sc_col.append(tmp)
            # compute rhs_sc contribution of bid block
            ri = permuted_rhs[bid].flatten()
            pi = lsolver.do_back_solve(ri)
            rhs_sc -= Bi.dot(pi)

        row = np.concatenate(sc_row)
        col = np.concatenate(sc_col)
        data = np.concatenate(sc_values)
        S = coo_matrix((data, (row, col)), shape=(nlp.nz, nlp.nz))
        S.sum_duplicates()

        # solve schur-complement
        delta_z = self._sc_solver.solve(S, rhs_sc, check_symmetry=False)

        # perform backsolves each block kkt
        permuted_sol = BlockVector(nblocks)
        for bid in range(nblocks):
            lsolver = self._lsolver[bid]
            ri = permuted_rhs[bid]
            rhs_i = ri - permuted_kkt[bid, nblocks].tocsr().dot(delta_z)
            xi = lsolver.do_back_solve(rhs_i)
            permuted_sol[bid] = xi

        # permute solution back
        sol = BlockVector(4)
        x_vals = []
        s_vals = []
        s_linking_vals = []
        yc_vals = []
        yc_linking_vals = []
        yd_vals = []
        yd_linking_vals = []

        for bid in range(nblocks):
            x_vals.append(permuted_sol[bid][0])
            s_vals.append(permuted_sol[bid][1])
            yc_vals.append(permuted_sol[bid][2])
            yd_vals.append(permuted_sol[bid][3])
            yc_linking_vals.append(permuted_sol[bid][4])
            yd_linking_vals.append(np.zeros(0))
            s_linking_vals.append(np.zeros(0))

        x_vals.append(delta_z)
        sol[0] = BlockVector(x_vals)
        sol[1] = BlockVector(s_vals + s_linking_vals)
        sol[2] = BlockVector(yc_vals + yc_linking_vals)
        sol[3] = BlockVector(yd_vals + yd_linking_vals)

        info = {'status': overall_status, 'delta_reg': val_reg_w, 'reg_iter': max_count_iter}
        return sol, info

    def reset_inertia_parameters(self):

        if self._inertia_params is not None:
            for l in self._inertia_params:
                l.reset()

    def __str__(self):
        return 'TwoStageStochasticSchurKKTSolver'

    def __repr__(self):
        return 'TwoStageStochasticSchurKKTSolver'
