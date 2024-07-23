# SmartReprojectionFactor

why add  key values and prior like this:



```c++
  graph.addPrior(X(1), Pose3::identity(), priorPoseNoise);
  initialEstimate.insert(X(0), Pose3::identity());

  // Bias prior
  graph.addPrior(B(1), imu.priorImuBias,
                                               imu.biasNoiseModel);
  initialEstimate.insert(B(0), imu.priorImuBias);

  // Velocity prior - assume stationary
  graph.addPrior(V(1), Vector3(0, 0, 0), imu.velocityNoiseModel);
  initialEstimate.insert(V(0), Vector3(0, 0, 0));
```



timeStamp can not be the same ?





terminate called after throwing an instance of 'std::out_of_range'
  what():  invalid key

should start after severay key ?



better way to do ?



TODO

1,set a time before update, but how to design the imu part ?

2,compare time with isam2

isam2 error :

```
terminate called after throwing an instance of 'std::invalid_argument'
  what():  Internal error, indices and factors passed into VariableIndex::remove are not consistent with the existing variable index
```



simple test using SmartFactorStereo_IMU.txt, lag = 1, 2 , some error in the end, lag = 3 works 





how to run debug in the source code ?



in FixedLagSmootherExampleVIO.cpp , what's the difference of adding prior to 0 and adding prior to 1 ?

## time

### batch

Time of each optimization 
0.001210281 0.003982229 0.015077225 0.025913083 0.034156282

### isam2

Time of each optimization 
0.001642546 0.004984075 0.005592726 0.006443112 0.02415292 





what I have done:

1,create FixedLagSmoother usage example for VIO data of gtsam example

2,tested and verified

3,find a simple gtsam-vio , adding FixedLagSmoother for it, done ? but need time to debug

4,if step.3 is done, provide a example to develop FixedLagSmoother of our mc-slam, 





## GenericStereoFactor



## IMU Factor



## Combined IMU Factor

how it works , in example and  our mc-slam

## RigFactor ?





# vio test 

for timesStamp， using real timeStamp, or window count (key number ?)



how to set LM params for smootherBatch ?



still not been solved:

```
terminate called after throwing an instance of 'std::invalid_argument'
  what():  Internal error, indices and factors passed into VariableIndex::remove are not consistent with the existing variable index
```

**need to check out the source code deeply, how they works** 





add timestamp, old variable outsied of the window size will marginalized, but without timestamp, seems not work ?(should just be isam ?)



window size, based on timestamp , or pose number ?

# mc-slam

imu key seems not been processed in FixedLagSmoother



why set  updateParams.removeFactorIndices = toBeRemovedFactorIndices;

```c++
ISAM2UpdateParams updateParams;
updateParams.newAffectedKeys = std::move(factorNewAffectedKeys);

updateParams.removeFactorIndices = toBeRemovedFactorIndices;
auto result = isam.update(graph, initialEstimate, updateParams);
```



adding prior, from yaml ?or online estimation



simulated frontend : same error : 

terminate called after throwing an instance of 'std::invalid_argument'
  what():  Internal error, indices and factors passed into VariableIndex::remove are not consistent with the existing variable index

gtsam print :

```
Current Timestamp: 9
Marginalizable Keys: b5 v5 x5 
Constrained Keys: b5(0)  b6(1)  b7(1)  b8(1)  b9(1)  v5(0)  v6(1)  v7(1)  v8(1)  v9(1)  x5(0)  x6(1)  x7(1)  x8(1)  x9(1)  
Bayes Tree After Update, Before Marginalization:
P( v7 b7 x8 x7 x6 v6 b6 )
 P( v8 b8 | b7 v7 x7 x8 )
  P( b9 x9 v9 | b8 v8 x8 )
 P( b5 x5 v5 | b6 v6 x6 )
END
Final Bayes Tree:
P( v7 b7 x8 x7 x6 v6 b6 )
 P( v8 b8 | b7 v7 x7 x8 )
  P( b9 x9 v9 | b8 v8 x8 )
END
```

## example 

```

IncrementalFixedLagSmoother::update() Start
Bayes Tree Before Update:
{Empty Tree}
END
Current Timestamp: 0.5
Marginalizable Keys: 
Constrained Keys: 
Bayes Tree After Update, Before Marginalization:
P( 250 500 )
 P( 0 | 250 )
END
Final Bayes Tree:
P( 250 500 )
 P( 0 | 250 )
END
IncrementalFixedLagSmoother::update() Finish
IncrementalFixedLagSmoother::update() Start
Bayes Tree Before Update:
P( 250 500 )
 P( 0 | 250 )
END
Current Timestamp: 0.75
Marginalizable Keys: 
Constrained Keys: 
Bayes Tree After Update, Before Marginalization:
P( 500 750 )
 P( 250 | 500 )
  P( 0 | 250 )
END
Final Bayes Tree:
P( 500 750 )
 P( 250 | 500 )
  P( 0 | 250 )
END
IncrementalFixedLagSmoother::update() Finish
IncrementalFixedLagSmoother::update() Start
Bayes Tree Before Update:
P( 500 750 )
 P( 250 | 500 )
  P( 0 | 250 )
END
Current Timestamp: 1
Marginalizable Keys: 
Constrained Keys: 
Bayes Tree After Update, Before Marginalization:
P( 750 1000 )
 P( 500 | 750 )
  P( 250 | 500 )
   P( 0 | 250 )
END
Final Bayes Tree:
P( 750 1000 )
 P( 500 | 750 )
  P( 250 | 500 )
   P( 0 | 250 )
END
IncrementalFixedLagSmoother::update() Finish
IncrementalFixedLagSmoother::update() Start
Bayes Tree Before Update:
P( 750 1000 )
 P( 500 | 750 )
  P( 250 | 500 )
   P( 0 | 250 )
END
Current Timestamp: 1.25
Marginalizable Keys: 
Constrained Keys: 
Bayes Tree After Update, Before Marginalization:
P( 1000 1250 )
 P( 750 | 1000 )
  P( 500 | 750 )
   P( 250 | 500 )
    P( 0 | 250 )
END
Final Bayes Tree:
P( 1000 1250 )
 P( 750 | 1000 )
  P( 500 | 750 )
   P( 250 | 500 )
    P( 0 | 250 )
END
IncrementalFixedLagSmoother::update() Finish
IncrementalFixedLagSmoother::update() Start
Bayes Tree Before Update:
P( 1000 1250 )
 P( 750 | 1000 )
  P( 500 | 750 )
   P( 250 | 500 )
    P( 0 | 250 )
END
Current Timestamp: 1.5
Marginalizable Keys: 
Constrained Keys: 
Bayes Tree After Update, Before Marginalization:
P( 1250 1500 )
 P( 1000 | 1250 )
  P( 750 | 1000 )
   P( 500 | 750 )
    P( 250 | 500 )
     P( 0 | 250 )
END
Final Bayes Tree:
P( 1250 1500 )
 P( 1000 | 1250 )
  P( 750 | 1000 )
   P( 500 | 750 )
    P( 250 | 500 )
     P( 0 | 250 )
END
IncrementalFixedLagSmoother::update() Finish
IncrementalFixedLagSmoother::update() Start
Bayes Tree Before Update:
P( 1250 1500 )
 P( 1000 | 1250 )
  P( 750 | 1000 )
   P( 500 | 750 )
    P( 250 | 500 )
     P( 0 | 250 )
END
Current Timestamp: 1.75
Marginalizable Keys: 
Constrained Keys: 
Bayes Tree After Update, Before Marginalization:
P( 1500 1750 )
 P( 1250 | 1500 )
  P( 1000 | 1250 )
   P( 750 | 1000 )
    P( 500 | 750 )
     P( 250 | 500 )
      P( 0 | 250 )
END
Final Bayes Tree:
P( 1500 1750 )
 P( 1250 | 1500 )
  P( 1000 | 1250 )
   P( 750 | 1000 )
    P( 500 | 750 )
     P( 250 | 500 )
      P( 0 | 250 )
END
IncrementalFixedLagSmoother::update() Finish
IncrementalFixedLagSmoother::update() Start
Bayes Tree Before Update:
P( 1500 1750 )
 P( 1250 | 1500 )
  P( 1000 | 1250 )
   P( 750 | 1000 )
    P( 500 | 750 )
     P( 250 | 500 )
      P( 0 | 250 )
END
Current Timestamp: 2
Marginalizable Keys: 
Constrained Keys: 
Bayes Tree After Update, Before Marginalization:
P( 1750 2000 )
 P( 1500 | 1750 )
  P( 1250 | 1500 )
   P( 1000 | 1250 )
    P( 750 | 1000 )
     P( 500 | 750 )
      P( 250 | 500 )
       P( 0 | 250 )
END
Final Bayes Tree:
P( 1750 2000 )
 P( 1500 | 1750 )
  P( 1250 | 1500 )
   P( 1000 | 1250 )
    P( 750 | 1000 )
     P( 500 | 750 )
      P( 250 | 500 )
       P( 0 | 250 )
END
IncrementalFixedLagSmoother::update() Finish
IncrementalFixedLagSmoother::update() Start
Bayes Tree Before Update:
P( 1750 2000 )
 P( 1500 | 1750 )
  P( 1250 | 1500 )
   P( 1000 | 1250 )
    P( 750 | 1000 )
     P( 500 | 750 )
      P( 250 | 500 )
       P( 0 | 250 )
END
Current Timestamp: 2.25
Marginalizable Keys: 0 
Constrained Keys: 0(0)  250(1)  500(1)  750(1)  1000(1)  1250(1)  1500(1)  1750(1)  2000(1)  2250(1)  
Bayes Tree After Update, Before Marginalization:
P( 2000 2250 )
 P( 1750 | 2000 )
  P( 1500 | 1750 )
   P( 1250 | 1500 )
    P( 1000 | 1250 )
     P( 750 | 1000 )
      P( 500 | 750 )
       P( 250 | 500 )
        P( 0 | 250 )
END
Final Bayes Tree:
P( 2000 2250 )
 P( 1750 | 2000 )
  P( 1500 | 1750 )
   P( 1250 | 1500 )
    P( 1000 | 1250 )
     P( 750 | 1000 )
      P( 500 | 750 )
       P( 250 | 500 )
END
IncrementalFixedLagSmoother::update() Finish
IncrementalFixedLagSmoother::update() Start
Bayes Tree Before Update:
P( 2000 2250 )
 P( 1750 | 2000 )
  P( 1500 | 1750 )
   P( 1250 | 1500 )
    P( 1000 | 1250 )
     P( 750 | 1000 )
      P( 500 | 750 )
       P( 250 | 500 )
END
Current Timestamp: 2.5
Marginalizable Keys: 250 
Constrained Keys: 250(0)  500(1)  750(1)  1000(1)  1250(1)  1500(1)  1750(1)  2000(1)  2250(1)  2500(1)  
Bayes Tree After Update, Before Marginalization:
P( 2250 2500 )
 P( 2000 | 2250 )
  P( 1750 | 2000 )
   P( 1500 | 1750 )
    P( 1250 | 1500 )
     P( 1000 | 1250 )
      P( 750 | 1000 )
       P( 500 | 750 )
        P( 250 | 500 )
END
Final Bayes Tree:
P( 2250 2500 )
 P( 2000 | 2250 )
  P( 1750 | 2000 )
   P( 1500 | 1750 )
    P( 1250 | 1500 )
     P( 1000 | 1250 )
      P( 750 | 1000 )
       P( 500 | 750 )
END
IncrementalFixedLagSmoother::update() Finish
IncrementalFixedLagSmoother::update() Start
Bayes Tree Before Update:
P( 2250 2500 )
 P( 2000 | 2250 )
  P( 1750 | 2000 )
   P( 1500 | 1750 )
    P( 1250 | 1500 )
     P( 1000 | 1250 )
      P( 750 | 1000 )
       P( 500 | 750 )
END
Current Timestamp: 2.75
Marginalizable Keys: 500 
Constrained Keys: 500(0)  750(1)  1000(1)  1250(1)  1500(1)  1750(1)  2000(1)  2250(1)  2500(1)  2750(1)  
Bayes Tree After Update, Before Marginalization:
P( 2500 2750 )
 P( 2250 | 2500 )
  P( 2000 | 2250 )
   P( 1750 | 2000 )
    P( 1500 | 1750 )
     P( 1250 | 1500 )
      P( 1000 | 1250 )
       P( 750 | 1000 )
        P( 500 | 750 )
END
Final Bayes Tree:
P( 2500 2750 )
 P( 2250 | 2500 )
  P( 2000 | 2250 )
   P( 1750 | 2000 )
    P( 1500 | 1750 )
     P( 1250 | 1500 )
      P( 1000 | 1250 )
       P( 750 | 1000 )
END
IncrementalFixedLagSmoother::update() Finish
IncrementalFixedLagSmoother::update() Start
Bayes Tree Before Update:
P( 2500 2750 )
 P( 2250 | 2500 )
  P( 2000 | 2250 )
   P( 1750 | 2000 )
    P( 1500 | 1750 )
     P( 1250 | 1500 )
      P( 1000 | 1250 )
       P( 750 | 1000 )
END
Current Timestamp: 3
Marginalizable Keys: 750 
Constrained Keys: 750(0)  1000(1)  1250(1)  1500(1)  1750(1)  2000(1)  2250(1)  2500(1)  2750(1)  3000(1)  
Bayes Tree After Update, Before Marginalization:
P( 2750 3000 )
 P( 2500 | 2750 )
  P( 2250 | 2500 )
   P( 2000 | 2250 )
    P( 1750 | 2000 )
     P( 1500 | 1750 )
      P( 1250 | 1500 )
       P( 1000 | 1250 )
        P( 750 | 1000 )
END
Final Bayes Tree:
P( 2750 3000 )
 P( 2500 | 2750 )
  P( 2250 | 2500 )
   P( 2000 | 2250 )
    P( 1750 | 2000 )
     P( 1500 | 1750 )
      P( 1250 | 1500 )
       P( 1000 | 1250 )
END
IncrementalFixedLagSmoother::update() Finish
```



## wrong one



# VIO test

## isam2, issue with 

```
terminate called after throwing an instance of 'std::invalid_argument'
  what():  Internal error, indices and factors passed into VariableIndex::remove are not consistent with the existing variable index
```



happened ins

```c++
void ISAM2::marginalizeLeaves(
    const FastList<Key>& leafKeysList,
    FactorIndices* marginalFactorsIndices,
    FactorIndices* deletedFactorsIndices)
```

Bayes tree print :

**first update:**

```
Current Timestamp: 9
Marginalizable Keys: b5 v5 x5 
Constrained Keys: b5(0)  b6(1)  b7(1)  b8(1)  b9(1)  v5(0)  v6(1)  v7(1)  v8(1)  v9(1)  x5(0)  x6(1)  x7(1)  x8(1)  x9(1)  
Bayes Tree After Update, Before Marginalization:
P( v7 b7 x8 x7 x6 v6 b6 )
 P( v8 b8 | b7 v7 x7 x8 )
  P( b9 x9 v9 | b8 v8 x8 )
 P( b5 x5 v5 | b6 v6 x6 )
END
for debug in isam2 
for debug in isam2 
for debug in isam2 
for debug in isam2 frontal7061644215716937733
for debug in isam2 frontal8646911284551352325
for debug in isam2 frontal8502796096475496453
eraseKeyTimestampMap
timestamp : 5 key : 7061644215716937733
eraseKeyTimestampMap
timestamp : 5 key : 8502796096475496453
eraseKeyTimestampMap
timestamp : 5 key : 8646911284551352325
Final Bayes Tree:
P( v7 b7 x8 x7 x6 v6 b6 )
 P( v8 b8 | b7 v7 x7 x8 )
  P( b9 x9 v9 | b8 v8 x8 )
END
IncrementalFixedLagSmoother::update() Finish
solve time cost = 0.14301 seconds.
```

**second update** 

```
Current Timestamp: 10
Marginalizable Keys: b6 v6 x6 
Constrained Keys: b6(0)  b7(1)  b8(1)  b9(1)  b10(1)  v6(0)  v7(1)  v8(1)  v9(1)  v10(1)  x6(0)  x7(1)  x8(1)  x9(1)  x10(1)  
Bayes Tree After Update, Before Marginalization:
P( x9 v9 b9 x10 v10 b10 )
 P( x8 v8 b8 | b9 v9 x9 )
  P( x7 v7 b7 | b8 v8 x8 x9 )
   P( x6 v6 | b7 v7 x7 x8 x9 )
    P( b6 | b7 v6 v7 x6 x7 )
END
for debug in isam2 
for debug in isam2 
for debug in isam2 
for debug in isam2 frontal8646911284551352326
for debug in isam2 frontal8502796096475496454
for debug in isam2 frontal7061644215716937734
terminate called after throwing an instance of 'std::invalid_argument'
  what():  Internal error, indices and factors passed into VariableIndex::remove are not consistent with the existing variable index
```



## LM, IndeterminantLinearSystemException happens

LM can do update for few frames



# Imu test

LM, isam2 both works

# VO test

LM can process 50 frames, then  "**IndeterminantLinearSystemException**"

isam2 process few frames, then "**IndeterminantLinearSystemException**"

# PGO test

## isam2 , IndeterminantLinearSystemException happens

```
IncrementalFixedLagSmoother::update() Finish
terminate called after throwing an instance of 'gtsam::IndeterminantLinearSystemException'
  what():  
Indeterminant linear system detected while working near variable
48500 (Symbol: 48500).
```

## LM can finish test of PGO



# new 

factor will be removed once they got new measurement (also new X variables)

find all the smartFactor related to the key to be removed 

factor remove, only for smartFactor, also for imuFactor ?

and need to add prior for the first variable of the window manually( "Add priors on all variables to fix indeterminant linear system")

what is slot ? is actually factor indices 



conclusion :

smartFactor should be constructed every times, although there has been constructed before, so delete them, and constructed again, this can make sure that all the factor has the new key(the key in the window)



Factor has ID, or what to put in : const gtsam::KeyVector &factorsToRemove = gtsam::FastVector<...>()

Factor has a ID, is Slot the Factor Index ? 





keep new measurement and old measurement

find out the graph adding update, and the relationship with smoother->getFactor



for the new measurement (new Factor), will always adding them, just if they were in the OldFactor, need to put them in the to_be_delete, so need flag to manage the 

```
using SmartFactorMap =
    gtsam::FastMap<LandmarkId, std::pair<SmartStereoFactor::shared_ptr, Slot>>;
```

if the Lmk_id has been 

old_smart_factors doesn't mean it is really old

  new_smart_factors_.insert(std::make_pair(lmk_id, new_factor));

  old_smart_factors_.insert(

​      std::make_pair(lmk_id, std::make_pair(new_factor, -1)));





​    **const size_t& slot = result.newFactorsIndices.at(i);**



```c++
      gtsam::ISAM2Result result;
      result = smootherISAM2.getISAM2Result();
      gtsam::FactorIndices newFactorIndex;
      newFactorIndex = result.newFactorsIndices;
      gtsam::NonlinearFactorGraph grapg_in_smoother = smootherISAM2.getFactors();
```



factor will get a unique ID, 



The indices of the newly-added factors, in 1-to-1 correspondence with the factors passed as `newFactors` to [ISAM2::update()](https://gtsam.org/doxygen/4.0.0/a03643.html#abd282f9b00477204ab09590993a974e6).

These indices may be used later to refer to the factors in order to remove them.



for index for vector， one is landmark， the other is index in whole Factor graph(smootherISAM2.getFactors())

why two index ? landmark ID can find smartFactor specifically, and slot is used to find the position of samrtFactor(to_be_delete) in the whole factor graph

so:

1, using landmark ID, we will construct new smartFactor everytime, (whether they have been constructed before), adding measurement, them put in newSmartFactor

2, using slot, -1 means not added to the graph yet, others is their index on the graph

3, where to work with newSmartFactor and oldSmartFactor

newSmartFactor. seems intuitive, just the new factor

oldSmartFactor, when construct newSmartFactor, also adding oldSmartFactor, and slot(-1), it just like the whole graph in smootherISAM2.getFactors(), will keep adding all the time, but newSmartFactor will be cleared



why delete, these factor has been created before, but for the key we may marginalized, so we will compare, if exists, just adding the "new version" of these factor

problem is that, landmark ID won't been used as the index in smoother's graph, so we need another index

, or how can I get the index of specific smartFactor[landmark] in the smoother's graph



is it possible of the same landmark , for single camera, be seen twice in single frame



## code issue

graph is empty, in all example, so not the problem of :

```
没有与参数列表匹配的 重载函数 "gtsam::NonlinearFactorGraph::push_back" 实例C/C++(304)
FixedLagSmootherExampleVIO_new.cpp(228, 29): 参数类型为: (const boost::shared_ptr<gtsam::SmartProjectionRigFactor<gtsam::PinholePose<gtsam::Cal3_S2>>>)
FixedLagSmootherExampleVIO_new.cpp(228, 29): 对象类型是: gtsam::NonlinearFactorGraph
```

graph.print can prove this



why result.newFactorsIndices is empty, after first smoother update

because I use batch and isam at the same time 



what's smootherISAM2_.getISAM2Result() , result.newFactorsIndices.at(i), the BatchFixedLagSmoother verison of it



isam2 issue, can not process much frames, using Batch version, but need to set a index for  result.newFactorsIndices.at(i)

getFactors( ) can save the 

use VariableIndex or can I use VariableSlots ?



is smartRigFactor has a key ? when printing the smartFactor, how is it looks like ? it is based on pose Key:

```
smartFactor[landmark]->printKeys();
Factor  x6 x7
Factor  x6 x7
Factor  x6 x7
```

but I want it contain only one pose key, so need to keep delete old smartFactor, and construct smartFactor for every frame



## using oldkey

since we can keep the smartFactor only be connected to one key, can we use the 

## using some existing api in gtsam

```c++
template <class FACTOR>
template <typename CONTAINER, typename>
FactorIndices FactorGraph<FACTOR>::add_factors(const CONTAINER& factors,
                                               bool useEmptySlots) {
  const size_t num_factors = factors.size();
  FactorIndices newFactorIndices(num_factors);
  if (useEmptySlots) {
    size_t i = 0;
    for (size_t j = 0; j < num_factors; ++j) {
      // Loop to find the next available factor slot
      do {
        if (i >= size())
          // Make room for remaining factors, happens only once.
          resize(size() + num_factors - j);
        else if (at(i))
          ++i;  // Move on to the next slot or past end.
        else
          break;  // We found an empty slot, break to fill it.
      } while (true);

      // Use the current slot, updating graph and newFactorSlots.
      at(i) = factors[j];
      newFactorIndices[j] = i;
    }
  } else {
    // We're not looking for unused slots, so just add the factors at the end.
    for (size_t i = 0; i < num_factors; ++i) newFactorIndices[i] = i + size();
    push_back(factors);
  }
  return newFactorIndices;
}
```



### 16:27

*old_smart_factors_* seems not save all the factor

*lmk_ids_of_new_smart_factors_* seems not the same size as *newFactorIndices_*



### 19:13

every time , we will generate a prior factor for the marginalized factor, so *old_smart_factors* is definitely not the index of slot in factor graph

### 20:14

gtsam::Fastmap seems werid, why size is 499 every time, using std::map to test

why the first factor be marginalized is 7, not 6 ?

to avoid the prior factor 6, being update twice, using a independent graph for the prior factor -> **NO **didn't works, so this is not a good idea, do not do single update for prior factor

### 20:39

feel the problem should be in ... update occurs, but no key be marginalized , there is no key to remove, only factor to remove, so may be so problem happens in key, not factor

### 9:18

using opertaer[] for the map, so that the value of old key can be overwritten

old_smart_factors stores the newest smartFactor for each LmkID, (only one smartFactor per LmkID)

the first iteration for newSmartFactor and old_smart_factors, seems right



no, should use insert, do not want change the old key-value pair, so that we can find the corresponding one in the history, and delete them,



slot in old_smart_factors have not been updated, maybe use class better, here just solve it by copy old_smart_factors



should marginalized X6 , why not ?



**some container size, can not be aligned** 

lmk_ids_of_new_smart_factors based on newSmartFactor, but is different 



### 13:45

cameraRig is fine

in GDB, can only visualize part of the pair in the container, so those container may be good to go now

so problem for now should look at why **marginalFactors.size() == 0**

because : marginalizableKeys : 1

   but : 

```
key : 8646911284551352326
marginalizableKeys : 8646911284551352326
```

it is the same one, so marginalizableKeys is actually X6

for now, I think the problem is on factors, not key

### 15:21

recall factor one more time

the logic of old_smart_factor is not correct, and initialize is also not good 

optimizer update untill second frame , can update successfully once 

```
Factor 0: LinearContainerFactor  keys = { x7 }
   Stored Factor
 keys: x7(6) 
```

marginalized X6, it is correct, next iteration should marginalize X7, but failed

### 19:23

a problem so far : 

```
              if (slot != -1) { // adding before, already exists // slot=-1 means the lmkID is new
                  if (smootherBatch.getFactors().exists(slot)){
```

but will go to :

```
                    std::cout << "slot not exist in smootherBatch.getFactors() " << std::endl;
```

which means, the slot managed in old_smart_factors may be wrong ？

another problem, if not got in to the sliding window, frames not enough, how to using the optimizer, and how to manage the related data structure

still think the problem could be in manage before optimization window, when the first key to marginalized, toDelete should not be related to the new_smart_factor, but to the x6 ?

so the logic of the code currently, 

### 20:41

how to set and use old_smart_factor seems not right

think about what functional I want old_smart_factor to achieve

### 00:16

what is the difference, of frame > 5 and frame > 6

again, refer to Kimera

## recap

### getSlotsIndices

### old_smart_factors

maybe all the structure for now is not correct

insert is correct, since we will update the slots, so it won't be the same everytime

why slot always -1?

### error

```
terminate called after throwing an instance of 'gtsam::IndeterminantLinearSystemException'
  what():  
Indeterminant linear system detected while working near variable
8646911284551352326 (Symbol: x6).
```

for now, the management of data seems good, using exceptioncheck to find the  IndeterminantLinearSystemException reason



### IndeterminantLinearSystemException

because we do not have the prior factor for the first Key, should check how it works in gtsam source code

```
 marginalFactors.size() : 0
debug******
        size: 0
```



how this works :

```
  GaussianFactorGraph::Eliminate getEliminationFunction() const {
    switch (linearSolverType) {
    case MULTIFRONTAL_CHOLESKY:
    case SEQUENTIAL_CHOLESKY:
      return EliminatePreferCholesky;

    case MULTIFRONTAL_QR:
    case SEQUENTIAL_QR:
      return EliminateQR;

    default:
      throw std::runtime_error(
          "Nonlinear optimization parameter \"factorization\" is invalid");
    }
  }
```



are you sure removedFactor (factorGraph) all related to key_to_be_marghinalized



what is the optimizer doing, before the marginalization window arrive 

slot, not the all the factor related to the key we want to marginalized 



delete factor from smoother every time, just not marginalize the oldest factor. when get into sliding window , starting marginalization 

### toDelte seems problem

designing philosophy is right, we do not want smart factor be connected with other Key(especially the key not in the sliding window), but when marginalization happens, toDelete is not all the factor connected with the oldest key to be marginalized, 

(but look at the key of these factor , seems the factor belong to the oldest key we want to marginalized ).................. 



we need to use toDelete, but toDelete is not the all satisfied our needs

or is the way I do has problem, using isam2 should be good ?

### do I need VioBackend::updateLandmarkInGraph ?



### verify data structure

it related to the frontend, some landmark, only looked by one frame, will keep in the graph, not actually delete !!!

find a new better to do delete

for now , we achieve every landmark only has one factor, but not achieving delete the all the factor for sliding window

(and I think the way Kimera work on toDelete, more compatible with their frontend)



find a way, landmark can have multiple factor in the window, but toDelete should generate from comparison between the last Key in the window, and the Key to be marginalized 

#### test1

based on the delete we have for now. adding all the factor supposed to be deleted to it

can we delete some factor before smoother update, then in smoother update , put the real factor we want to delete

just remove, not marginalized 

so 1, toRemove, just as the same toDelete before

2, toDelete, all the factor related to the key to be marginalized 

modify gtsam 

some factor has been removed, can the slot of them still in toDelete

when a factor has been deleted , or removed, is it still be as a nullptr in the graph

......not good



reset(), remove()



so far, I think Kimera's way maybe right, the problem for me is some the factor for the key to be marginalized, still have some connection with other factor, aka, not really achieving each factor, only observe single key frame



slot should not be that big, since new factor will come to the slot has been removed

and in update(), insert factor firstly, then comes to remove, so slot should be ?



from here :

```c++
void BatchFixedLagSmoother::insertFactors(
    const NonlinearFactorGraph& newFactors) {
  for(const auto& factor: newFactors) {
    Key index;
    // Insert the factor into an existing hole in the factor graph, if possible
    if (availableSlots_.size() > 0) {
      index = availableSlots_.front();
      availableSlots_.pop();
      factors_.replace(index, factor);
    } else {
      index = factors_.size();
      factors_.push_back(factor);
    }
    // Update the FactorIndex
    for(Key key: *factor) {
      factorIndex_[key].insert(index);
    }
  }
}
```

we can know that the new added factor will fill the slot of the removed factor, so ...



```c++
    /** delete factor without re-arranging indexes by inserting a NULL pointer */
    void remove(size_t i) { factors_[i].reset();}
```





anyway...... I can conclude that slot is wrong for now

```c++
  const FactorIndices& getNewSlots() const {
    return newSlots_;    
  }
  
  void clearNewSlots() {
    newSlots_.clear();
  }
```



feel like not the newly added factors ?



```c++
void BatchFixedLagSmoother::removeFactors(
    const set<size_t>& deleteFactors) {
  for(size_t slot: deleteFactors) {
    if (factors_.at(slot)) {
      // Remove references to this factor from the FactorIndex
      for(Key key: *(factors_.at(slot))) {
        factorIndex_[key].erase(slot);
      }
      // Remove the factor from the factor graph
      factors_.remove(slot);
      // Add the factor's old slot to the list of available slots
      availableSlots_.push(slot);
    } else {
      // TODO: Throw an error??
      cout << "Attempting to remove a factor from slot " << slot
          << ", but it is already nullptr." << endl;
    }
  }
}
```

**deleteFactors** only changes when removeFactors, which only be called in marginalize



newSlots_ is only  for smartFactor ? only counting  smartFactor 



### test2

1, why prior factor related to all the variable, not only the last in the window







### dm-vio

```c++
void dmvio::extractKeysToMarginalize(const gtsam::NonlinearFactorGraph& graph, gtsam::NonlinearFactorGraph& newGraph,
                                     gtsam::NonlinearFactorGraph& marginalizedOutGraph,
                                     gtsam::FastSet<gtsam::Key>& setOfKeysToMarginalize,
                                     gtsam::FastSet<gtsam::Key>& connectedKeys)
{
    for(size_t i = 0; i < graph.size(); i++)
    {
        gtsam::NonlinearFactor::shared_ptr factor = graph.at(i);

        gtsam::FastSet<gtsam::Key> set_of_factor_keys(factor->keys());

        gtsam::FastSet<gtsam::Key> intersection;

        std::set_intersection(setOfKeysToMarginalize.begin(), setOfKeysToMarginalize.end(),
                              set_of_factor_keys.begin(), set_of_factor_keys.end(),
                              std::inserter(intersection, intersection.begin()));

        if(!intersection.empty())
        {
            std::set_difference(set_of_factor_keys.begin(), set_of_factor_keys.end(),
                                setOfKeysToMarginalize.begin(), setOfKeysToMarginalize.end(),
                                std::inserter(connectedKeys, connectedKeys.begin()));

            marginalizedOutGraph.add(factor);
        }else
        {
            newGraph.add(factor);
        }
    }
}
```

if I traverse the graph, to find the factor to romove, and I keep generating new samrtFactor as before(should not smartFactor will observe multiple keys), so I don't need old_smart_fatcors_ ?

### one more recall

factorsToRemove is just optional, not relevant to marginalization, just make sure each smartFactor only have one key

cmake -D WITH_VTK=ON -D WITH_OPENGL=ON -D WITH_QT=ON -D WITH_TBB=ON -D BUILD_opencv_viz=ON cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/home/jason/DPGO/catkin_ws/ThirdParty/opencv/install

# new 

## smart factor

is it because we have rig factor, even one pose, but some has observed by two cameras, so still got triangulation works ?

before window size come, won't delete any factor





I have keep the measurement : 

```
                    RigFactor::shared_ptr newFactor(new RigFactor(*old_factor));
```

the smart factor of the same landmark will be created multiple times, so we need remove , so the slot matters

remove the smart factor which has the same measurement as before, avoid a landmark has been added multiple times,  

why the ptr in old_smart_factor only have one key

## why need remove

one landmark has been constructed for smart factor twice, so need to remove from factor graph in the optimizer

## how to choose the factor to marginalize

factor involved with the key to be marginalized, will be marginalized out 

## how to update slot



## VIO



# meeting with Dave

1, the general formulation of the problem

2, the fastest way to achieve it  (refer to Kimera)

3, smart factor makes any difference ? (different between PGO, generic reprojection, smart factor) 

4, ISAM2 and LM for now

5, the way to achieve LM, 

# indeterminate exception

## isam 

newFactor->printKeys() seems normal

## LM

newFactor->printKeys() seems not normal, so the problem is in slot management



# twice measurement for smart factor

do not call update, until third frame 



why erase, even it is new:

```
                    // old_smart_factors.old_smart_factors_.erase(old_smart_factors_it);
                    std::cout << "slot not exist in smootherBatch.getFactors() " << std::endl;
```



seems going into a dead-end ?



how is the measurement been saved when copy constructing samrt 



# old logs

when you reading the measurement from txt, you will know the window size of next optimization, and only adding the factors involved  with key inside the window



# try-catch recovery

## factor check 

```
result:
point =  1808.85
-2301.43
-1097.33

result:
no point, status = 2


```

where can we check the quality for those triangulated landmark

1, like Kimera doing,  adding TriangulationResult checking

2, using generic projection factor

3, choose valid landmark, like (mc-slam ?) is doing 



new error 

```
terminate called after throwing an instance of 'gtsam::CheiralityException'
  what():  CheiralityException
```

try as Kimera, also adding prior in catch ? 



### triangulation result check

before first/second... all update : all are invalid (degenerate)

after first/second...all update : all are invalid

### linearized point

how to check ?

1, when I print the factor, which caused cholesky failed, from the smoother, the triangulation result seems invalid, why it is invalid, because the observed poses of this landmark changed a lot ? so we should look at the triangulation result of this landmark at other place ? like before linearized, or just the initial value

### recursively try-catch

need to copy smoother every time, how to do it recursively ?  **solved**

  

# integrating codebase

## standalone test

for now, just VO,  need adding IMUFactor ?

generate logs, plus IMU raw measurement ?, how to generate ?

## plan

### finish standalone test with VIO data

benchmark , run in bag with groundtruth

speed test

1, get online bias estimation from ros bag

2, finish reading imu data, create imu factor, adding/managing  imu factor, *update imu preintegration  from  current estimate result*

 find corresponding data period, reading imu in batch ,

(update imu preintegration a little consued, only update once ?)



need to remove "b" and "v" manually ? or like processing between factor, delete automatically ?

how to get the initial value of "b" and "v", should from the result of p

getting the initial bias from online estimation 



how we get initials for "v" and "b" in mc-slam

how to process initial stage

### moving code

#### probably issue

difference on the workflow of the code

moving should be compatible, not couple with frontend

1, can I finished a unit test , before moving it ?

2, how to save the final traj ?

3, processing time, will FixedLagSmoother with isam be faster ?

#### workflow of the current code

first, recall the workflow of FixedLagSmoother



#### design

new factor graph

initialEstimate , the same

timeStamp



line 3110 in Backend.cpp

what's IsamUpdateParam



try-catch recovery, for the normal isam works, but not find recovery works ?



two problem :

1, how to set up breakpoint in clion, but not disturb that function

2, I don't know is this really working ? or I delete the loop closure factor ?

Now, it is catch is working, but delete factors won't make optimization better ? why

1, how the origin isam2 work, is the same exception at the same place ?

2, 

normal, failed at : added measurement for lid 1462 to x204 for cam 0

# TODO

1, check kri2 calibration

2, check cholesky function

3, what's the body frame we are using ?



what's the meaning of Hessian is 0



standalone test for isam2, is always ignoring



is the slot right, do we really delete those factors ?

so check slot. using bookkeeping of previous version 



## reply

1， I can not run kri2, it will always fail , it will never go inside if, so no factors have been deleted

​                                    boost::tie(maxrank, success) = choleskyCareful(Ab_.matrix());

​                                    if (!(success || maxrank == hessian_factor.rows() - 1))

But I can run combined kri2 data, also with 3 cameras...I checked the calibration, the params, I 



checked inside, I found Hessian is a 0 , (for every factor), so success is true, but maxrank is 0. Triangulation result is very weird weird. So I prefer data/calibration/extrinsic is wrong ?



2, change lag size, all the test before is done with lag  = 5,  when lag is 10, really slow, because factors deleted in catch didn't make update() success, so need multiple times going to cholesky check. when lag is big, definitely more poses will be optimized, I don't know when the lag size is small, how is the accuracy





