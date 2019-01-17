from pyomo.core.base.constraint import Constraint
from pyomo.core.base.sos import SOSConstraint
from pyomo.core.base.objective import Objective
from pyomo.repn.standard_repn import generate_standard_repn
from pyomo.core.kernel.component_map import ComponentMap
from pyomo.core.base import SymbolMap, NumericLabeler, TextLabeler
import pyomo.core.expr.numvalue as numvalue
import logging


logger = logging.getLogger(__name__)


def _check_feasibility_of_fixed_constraint(con):
    """

    Parameters
    ----------
    con: pyomo.core.base.constraint.Constraint
    """
    assert numvalue.is_fixed(con.body)
    body_val = numvalue.value(con.body)
    need_warning = False
    if con.has_lb():
        if numvalue.value(con.lower) > body_val:
            need_warning = True
    if con.has_ub():
        if numvalue.value(con.upper) < body_val:
            need_warning = True
    if need_warning:
        logger.warning('Body of constraint {0} is fixed to an infeasible value.'.format(con.name))


class LPWriter(object):
    def __init__(self):
        self._pyomo_var_to_lp_string_map = ComponentMap()
        self._pyomo_con_to_lp_string_map = ComponentMap()
        self._pyomo_range_con_to_lp_string_map = ComponentMap()
        self._symbol_map = SymbolMap()
        self._var_labeler = TextLabeler()
        self._con_labeler = TextLabeler()

    def _generate_lp_string_for_expr(self, expr):
        """
        Parameters
        ----------
        expr: pyomo.core.expr.expr_pyomo5.ExpressionBase

        Returns
        -------
        lp_string: str
        repn: pyomo.repn.standard_repn.StandardRepn
        """
        repn = generate_standard_repn(expr=expr, quadratic=True)

        if (not repn.linear_vars) and (not repn.quadratic_vars):
            return None  # the body of the constraint is fixed

        if repn.nonlinear_vars:
            raise ValueError('Cannot write a valid lp file with nonlinear expression {0}'.format(str(expr)))

        lp_string = ''
        for i, v in enumerate(repn.linear_vars):
            if v not in self._pyomo_var_to_lp_string_map:
                self._add_var(v)
            coef = repn.linear_coefs[i]
            if coef >= 0:
                lp_string += '+ '
            lp_string += str(coef) + ' ' + self._symbol_map.getSymbol(v) + '\n'
        if repn.quadratic_vars:
            lp_string += '+ [\n'
            for i, v in enumerate(repn.quadratic_vars):
                v1, v2 = v
                if v1 not in self._pyomo_var_to_lp_string_map:
                    self._add_var(v1)
                coef = repn.quadratic_coefs[i]
                if coef >= 0:
                    lp_string += '+ '
                if v1 is v2:
                    lp_string += str(coef) + ' ' + self._symbol_map.getSymbol(v1) + ' ^2\n'
                else:
                    if v2 not in self._pyomo_var_to_lp_string_map:
                        self._add_var(v2)
                    lp_string += (str(coef) + ' ' + self._symbol_map.getSymbol(v1) +
                                  ' * ' + self._symbol_map.getSymbol(v2) + '\n')
            lp_string += ']\n'

        return lp_string, repn

    def generate_lp_string_for_range_con(self, con):
        """
        Parameters
        ----------
        con: pyomo.core.base.constraint.Constraint

        Returns
        -------
        lp_string1: str
        lp_string2: str
        """
        con_name = self._symbol_map.getSymbol(con, labeler=self._con_labeler)

        lp_string1, repn = self._generate_lp_string_for_expr(con.body)
        if lp_string1 is None:  # body is fixed
            _check_feasibility_of_fixed_constraint(con)
            return None, None
        lp_string2 = lp_string1

        lp_string1 += '>= ' + str(numvalue.value(con.lower) - repn.constant)
        lp_string2 += '<= ' + str(numvalue.value(con.upper) - repn.constant)
        lp_string1 = con_name + '_l:\n' + lp_string1 + '\n'
        lp_string2 = con_name + '_u:\n' + lp_string2 + '\n'

        return lp_string1, lp_string2

    def generate_lp_string_for_con(self, con):
        """

        Parameters
        ----------
        con: pyomo.core.base.constraint.Constraint

        Returns
        -------
        lp_string: str
        """
        con_name = self._symbol_map.getSymbol(con, labeler=self._con_labeler)
        lp_string, repn = self._generate_lp_string_for_expr(con.body)
        if lp_string is None:  # body is fixed
            _check_feasibility_of_fixed_constraint(con)
            return None

        if con.equality:
            lp_string += '= ' + str(numvalue.value(con.lower) - repn.constant)
        elif con.has_lb() and con.has_ub():
            raise ValueError('LPWriter.generate_lp_string_for_range_con should be used for range constraints; '
                             'Constraint of interest is: {0}'.format(con_name))
        elif con.has_lb():
            lp_string += '>= ' + str(numvalue.value(con.lower) - repn.constant)
        elif con.has_ub():
            lp_string += '<= ' + str(numvalue.value(con.upper) - repn.constant)
        else:
            return None
        lp_string = con_name + ':\n' + lp_string + '\n'
        return lp_string

    def _add_constraint(self, con):
        if not con.active:
            return None

        con_name = self._symbol_map.getSymbol(con, labeler=self._con_labeler)

        if con.equality or (not con.has_lb()) or (not con.has_ub()):
            lp_string = self.generate_lp_string_for_con(con)
            if lp_string is None:
                self._symbol_map.removeSymbol(con)
                return None
            self._pyomo_con_to_lp_string_map[con] = lp_string
        else:
            lp_string1, lp_string2 = self.generate_lp_string_for_range_con(con)
            if lp_string1 is None:
                self._symbol_map.removeSymbol(con)
                return None
            self._pyomo_range_con_to_lp_string_map[con] = (lp_string1, lp_string2)

    def _add_sos_constraint(self, con):
        raise NotImplementedError('SOS constraints are not yet supported by LPWriter')

    def _add_var(self, v):
        v_name = self._symbol_map.getSymbol(v, labeler=self._var_labeler)
        if v.has_lb():
            if v.has_ub():
                lp_string = str(numvalue.value(v.lb)) + ' <= ' + v_name + ' <= ' + str(numvalue.value(v.ub)) + '\n'
            else:
                lp_string = v_name + ' >= ' + str(numvalue.value(v.lb)) + '\n'
        elif v.has_ub():
            lp_string = v_name + ' <= ' + str(numvalue.value(v.ub)) + '\n'
        else:
            lp_string = v_name + ' free\n'
        self._pyomo_var_to_lp_string_map[v] = lp_string

    def _set_instance(self, model):
        self._add_block(model)

    def _set_objective(self, obj):
        raise NotImplementedError('Lazy')

    def _add_block(self, block):
        for sub_block in block.block_data_objects(descend_into=True, active=True, sort=True):
            for con in sub_block.component_data_objects(ctype=Constraint, descend_into=False, active=True, sort=True):
                self._add_constraint(con)

            for con in sub_block.component_data_objects(ctype=SOSConstraint, descend_into=False, active=True, sort=True):
                self._add_sos_constraint(con)

            obj_counter = 0
            for obj in sub_block.component_data_objects(ctype=Objective, descend_into=False, active=True):
                obj_counter += 1
                if obj_counter > 1:
                    raise ValueError("LPWriter does not support multiple objectives.")
                self._set_objective(obj)

    def write(self, model, filename):
        self._set_instance(model)
