#!/usr/bin/env python
import rospy

from smach import State
from random import randrange

class RandomSelectionFromPoolState(State):
    ''' pool is a list with the objects to be selected. Max_tries the maximum number of random tries before giving up and repeating the last one
        output_key selected_item is the randomly selected item from the list.
    '''
    def __init__(self, pool, max_tries=10):
        State.__init__(self, outcomes=['succeeded'], output_keys=['selected_item'])
        self._pool = pool
        self._lenpool = len(pool)
        self._lastSelection = None
        self._MAX_TRIES = max_tries


    def execute(self, userdata):
        selection = randrange(self._lenpool)
        if self._lenpool > 1:
            i = 0
            while (selection == self._lastSelection) and (i < self._MAX_TRIES):
                selection = randrange(self._lenpool)
                i += 1
            if i >= self._MAX_TRIES:
                rospy.logwarn('Maximum number of random tries (%d) reached! Consider augmenting it...' % self._MAX_TRIES)

        self._lastSelection = selection
        userdata.selected_item = self._pool[selection]
        return 'succeeded'

