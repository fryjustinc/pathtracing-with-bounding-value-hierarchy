BIH = function (dimensions, leafSizeMin, leafSizeMax, buildAlgorithm) {
        this._dimensions = dimensions || this._dimensions; 
        // The optimal leaf size is dependent on the user agent and model size
        // Chrome : 1-11  ?? Who knows ??
        // Firefox: ~3
        // Opera  : ~?
        // IE     : ~?

        this._minLeaf = leafSizeMin || this._minLeaf; // Minimum leaf size
        this._maxLeaf = leafSizeMax || this._maxLeaf; // Maximum leaf size

        this.treeBuilder = this.generateSAHBuilder(this);

        this._T = null; // The tree's root
        this.i = null;  // The tree's AABB
        this.chachedHelpers = [];
        this.segmentHelpers = this.generateSegmentHelpers(this._dimensions);
        this.nodeHelpers= this.generateNodeHelpers(this._dimensions);
    };

    BIH.prototype = {

        constructor : BIH,

        _name: "BIH",
        _dimensions: 3,
        _minLeaf: 2,
        _maxLeaf: 4,
        _kT: 50,  /* = _kT - Cost per node-traversal */
        _kI: 50,  /* = _kI - Cost per intersection test */
        _kO: 1.2, /* = _kO - Cost savings for *empty* overlapped area (higher = better) */
        _kB: 1,   /* = _kB - Cost savings for balanced splits (lower = better) */

        
        generateNodeHelpers: function (dimensions) {
            var cachedHelpers = [];
    // Cache helpers for each dimension since we only need to make 1
			if(dimensions in cachedHelpers)
				return cachedHelpers[dimensions];

    return (cachedHelpers[dimensions] = {
        _Dimensions: dimensions,

        makeArrayOfNodes : function(arrayOfElements) {
            var i = arrayOfElements.length,
                arrayOfNodes = [],
                element;

            while(i--) {
                element = arrayOfElements[i];
                arrayOfNodes.push(this.makeNodeFromElement(element));
            }
            return arrayOfNodes;
        },

        makeArrayOfNodesAsync : function(arrayOfElements, progressCallback, finishedCallback) {
            var totalElementsCount = arrayOfElements.length,
                i = arrayOfElements.length,
                arrayOfNodes = [];
            if(progressCallback) progressCallback({phase:"Generating nodes from primitives...", percent: 0});
            process.nextTick(makeNodes);

            function makeNodes(){
                var startTime = Date.now(),
                    element,
                    aabb,
                    weight;
                while(i--) {
                    element = arrayOfElements[i];

                    arrayOfNodes.push(this.makeNodeFromElement(element));

                    if(Date.now() - startTime > 1000) {
                        if(progressCallback)
                            progressCallback({percent: (totalElementsCount - i) / totalElementsCount * 100});
                        return process.nextTick(makeNodes);
                    }
                }
                if(progressCallback) progressCallback({percent: 100});
                return finishedCallback(arrayOfNodes);
            }
        },
       makeNodeFromElement : function(element) {
			var aabb,
			    weight,
			    intersect,
			    overlaps,
			    contains,
			    contained;
			aabb =element.aabb;

			weight = 1;

			intersect = function (ray, intersectInfo) {
			    var rs = element.aabb.intersectWithRay(ray.toIntervals());
			    if (rs) {
			        var rv = Vec(rs[0].a, rs[1].a, rs[2].a);
			        var t = Vec.sub(rv, ray.position);
			        t = Math.sqrt(t.x * t.x + t.y * t.y + t.z * t.z)
			        if (t < ray.maxT) {
			            ray.maxT = t;
			            intersectInfo.shape = this;
			            intersectInfo.isHit = true;
			            intersectInfo.position = rv;
			        }
			    }
			};

			overlaps = function (aabb2, returnArray) {
			    if (aabb2.overlaps(element.aabb)) returnArray.push(this);
			};

			contains = function (aabb2, returnArray) {
			    if (element.aabb.contains(aabb2)) returnArray.push(this);
			};

			contained = function (aabb2, returnArray) {
			    if (element.aabb.contained(aabb2)) returnArray.push(this);
			};

            return {
                i: aabb,
                w: weight,
                o: element,
                iFn: intersect,
                oFn: overlaps,
                csFn: contains,
                cdFn: contained
            };
        },
        makeWeight : function(nodes) {
            var w = 0,
                i = nodes.length;
            while(i--) {
                w += nodes[i].w;
            }
            return (w);
        },

        makeMBV : function(nodes) {
            var i = nodes.length,
                intervals;

            if (i < 1) return new AABB();

            intervals = nodes[0].i.clone();

            while (i-->1) {
                intervals.expandByAABB(nodes[i].i);
            }

            return (intervals);
        },

        makeSortedArrays : function(arrayOfNodes) {
            var destinationArrays = [],
                numberOfElements = arrayOfNodes.length,
                numberOfAxis = this._Dimensions, // Length of bounding box array
                sortedArray = [],
                sortFunction = function(a, b) {
                    return (b.i.min[numberOfAxis] - a.i.min[numberOfAxis]);
                }

            while(numberOfAxis-->0) {
                sortedArray = arrayOfNodes.slice(0);
                sortedArray.sort(sortFunction);
                destinationArrays[numberOfAxis] = sortedArray;
                sortedArray = [];
            }

            return destinationArrays;
        },

        splitSortedNodeArrays : function(sortedArraysOfNodes, bestAxis, bestIndex, leftPlane, rightPlane) {
            var numberOfAxis = sortedArraysOfNodes.length, // Length of bounding box array
                numberOfElements = 0,
                //we make 2 * # of axis lists (so 2 lists of length 3)
                destinationArrays = [[], []],
                leftArray,
                rightArray,
                elementArray,
                element,
                totalnumberOfElements;

            // First, split the best-fit axis
            rightArray = sortedArraysOfNodes[bestAxis].splice(0, bestIndex);
            leftArray = sortedArraysOfNodes[bestAxis];//.slice(bestIndex);

            destinationArrays[0][bestAxis] = leftArray;
            destinationArrays[1][bestAxis] = rightArray;

            while(numberOfAxis-->0) {
                if(numberOfAxis == bestAxis) continue;

                leftArray = [];
                rightArray = [];

                elementArray = sortedArraysOfNodes[numberOfAxis];
                numberOfElements = elementArray.length;
                totalnumberOfElements = numberOfElements - 1;

                while(numberOfElements-->0) {
                    element = elementArray[totalnumberOfElements - numberOfElements];

                    // We sort elements based on being outside of the left or right planes first
                    // Only if they are in the overlapped region do we perform the more expensive
                    // search for them to decide which array they should go into.
                    if(element.i.min[bestAxis] < rightPlane) {
                        leftArray.push(element);
                    } else if(element.i.max[bestAxis] > leftPlane) {
                        rightArray.push(element);
                    } else if( destinationArrays[0][bestAxis].indexOf(element) >= 0) {
                        leftArray.push(element);
                    } else {
                        rightArray.push(element);
                    }
                }
                destinationArrays[0][numberOfAxis] = leftArray;
                destinationArrays[1][numberOfAxis] = rightArray;
            }
            return destinationArrays;
        }
    });
    },
        generateSegmentHelpers: function (dimensions) {
            var cachedHelpers = [];
        // Cache helpers for each dimension since we only need to make 1
        if(dimensions in cachedHelpers)
            return cachedHelpers[dimensions];

    return (cachedHelpers[dimensions] = {
        _Dimensions: dimensions,

        cloneSegment : function(rs) {
            // if(tdist < 0 ) throw "What!";
            var retRS = new Array(this._Dimensions),
                i = 0;
            for (; i < this._Dimensions; i++) {
                retRS[i] = {
                    a: rs[i].a,
                    b: rs[i].b
                };
            }
            return retRS;
        },

        clipSegmentStart : function(rs, axis, splitPlane) {
            var tdist = (splitPlane - rs[axis].a) / (rs[axis].b - rs[axis].a),
                retRS = new Array(this._Dimensions),
                i = 0;

            for (; i < this._Dimensions; i++) {
                if (i !== axis) {
                    retRS[i] = {
                        a: rs[i].a + (rs[i].b - rs[i].a) * tdist,
                        b: rs[i].b
                    };
                } else {
                    retRS[i] = {
                        a: splitPlane,
                        b: rs[i].b
                    };
                }
            }
            return retRS;
        },

        clipSegmentStartInPlace : function(rs, axis, splitPlane) {
            var tdist = (splitPlane - rs[axis].a) / (rs[axis].b - rs[axis].a),
                i = 0;
            for (; i < this._Dimensions; i++) {
                if (i !== axis) {
                    rs[i].a = rs[i].a + (rs[i].b - rs[i].a) * tdist;
                } else {
                    rs[i].a = splitPlane;
                }
            }
        },

        clipSegmentEnd : function(rs, axis, splitPlane) {
            var tdist = (splitPlane - rs[axis].a) / (rs[axis].b - rs[axis].a),
                retRS = new Array(this._Dimensions),
                i = 0;
            for (; i < this._Dimensions; i++) {
                if (i !== axis) {
                    retRS[i] = {
                        a: rs[i].a,
                        b: rs[i].a + (rs[i].b - rs[i].a) * tdist
                    };
                } else {
                    retRS[i] = {
                        a: rs[i].a,
                        b: splitPlane
                    };
                }
            }
            return retRS;
        },

        clipSegmentEndInPlace : function(rs, axis, splitPlane) {
            var tdist = (splitPlane - rs[axis].a) / (rs[axis].b - rs[axis].a),
                i = 0;
            for (; i < this._Dimensions; i++) {
                if (i !== axis) {
                    rs[i].b = rs[i].a + (rs[i].b - rs[i].a) * tdist;
                } else {
                    rs[i].b = splitPlane;
                }
            }
        },
				
        copySegmentStartFromSegmentStart : function(destination, source) {
            for(var i = 0; i < this._Dimensions; i++)
                destination[i].a = source[i].a;
        },

        copySegmentStartFromSegmentEnd : function(destination, source) {
            for(var i = 0; i < this._Dimensions; i++)
                destination[i].a = source[i].b;
        },

        copySegmentEndFromSegmentStart : function(destination, source) {
            for(var i = 0; i < this._Dimensions; i++)
                destination[i].b = source[i].a;
        },

        copySegmentEndFromSegmentEnd : function(destination, source) {
            for(var i = 0; i < this._Dimensions; i++)
                destination[i].b = source[i].b;
        },

        trimSegmentInPlace : function(bestSegment, raySegment, axis) {
            var bestDir = 1,
                rayDir = 1,
                bestA = Math.min(bestSegment[axis].a, bestSegment[axis].b),
                bestB = Math.max(bestSegment[axis].a, bestSegment[axis].b),
                rayA = Math.min(raySegment[axis].a, raySegment[axis].b),
                rayB = Math.max(raySegment[axis].a, raySegment[axis].b);
            if(bestA !== bestSegment[axis].a)
                bestDir = -1;
            if(rayA !== raySegment[axis].a)
                rayDir = -1;

            if(bestA > rayB || bestB < rayA) {
                // bestSegment and raySegment DO NOT overlap
                return false;
            } else if(rayA >= bestA && rayB <= bestB) {
                // raySegment is contained completely in bestSegment
                return true;
            } else if(bestA >= rayA && bestB <= rayB) {
                // bestSegment is contained completely in raySegment
                this.copySegmentStartFromSegmentStart(raySegment, bestSegment);
                this.copySegmentEndFromSegmentEnd(raySegment, bestSegment);
                return true;
            } else if(rayA >= bestA && rayA <= bestB) {
                // raySegment starts inside bestSegment (and must exit outside it!)
                if(rayDir == 1) { 
                    if(bestDir == 1) {
                        this.copySegmentEndFromSegmentEnd(raySegment, bestSegment);
                    } else {
                        this.copySegmentEndFromSegmentStart(raySegment, bestSegment);
                    }
                } else {
                    if(bestDir == 1) {
                        this.copySegmentStartFromSegmentEnd(raySegment, raySegment);
                        this.copySegmentEndFromSegmentEnd(raySegment, bestSegment);
                    } else {
                        this.copySegmentStartFromSegmentEnd(raySegment, raySegment);
                        this.copySegmentEndFromSegmentStart(raySegment, bestSegment);
                    }
                }
                return true;
            } else {
                // bestSegment starts inside raySegment (and must exit outside it!)
                if(bestDir == 1) {
                    if(rayDir == 1) {
                        this.copySegmentStartFromSegmentStart(raySegment, bestSegment);
                    } else {
                        this.copySegmentEndFromSegmentStart(raySegment, raySegment);
                        this.copySegmentStartFromSegmentStart(raySegment, bestSegment);
                    }
                } else {
                    if(rayDir == 1) {
                        this.copySegmentStartFromSegmentEnd(raySegment, bestSegment);
                    } else {
                        this.copySegmentEndFromSegmentStart(raySegment, raySegment);
                        this.copySegmentStartFromSegmentEnd(raySegment, bestSegment);
                    }
                }
                return true;
            }
        },

        setBestSegment : function(bestSegment, newEndpoint) {
            for(var i = 0; i< this._Dimensions; i++)
                bestSegment[i].b = newEndpoint[i];
        }
    }
    );
},
        // ALL nodes only have one-letter variables to save space in the event that the tree is serialized.
        // TODO: Allow the tree to be serialized. :)
        generateSAHBuilder : function(tree) {
            return {
                _name : "SAH",
                _tree : tree,

                // Based on SAH
                // TODO: Make _excluded_ region count towards cost (as a bonus).
                calculateSplitCost : function(numberOfAxis, leftPlane, rightPlane, leftCount, rightCount, leftAABB, rightAABB, parentSurfaceArea) {
                    var t = this._tree._dimensions,
                        s,
                        leftSurfaceArea = 0,
                        rightSurfaceArea = 0,
                        overlapSurfaceArea = 0,
                        overlapWidth = leftPlane - rightPlane,
                        doesOverlap = (overlapWidth > 0 ? true : false),
                        SAH,
                        b;

                    overlapWidth = Math.abs(overlapWidth);

                    leftSurfaceArea = leftAABB.getSurfaceArea();
                    rightSurfaceArea = rightAABB.getSurfaceArea();
                    while(t-->0) {
                        s = t - 1;
                        if(s < 0) s = this._tree._dimensions - 1;
                        if(t == numberOfAxis || s == numberOfAxis) {
                            overlapSurfaceArea   += 2 * (t == numberOfAxis ? overlapWidth : leftAABB.getLength(t)) 
                                                      * (s == numberOfAxis ? overlapWidth : rightAABB.getLength(s));
                        }
                    }

                    if(doesOverlap)
                        SAH = this._tree._kT + this._tree._kI * ( (leftSurfaceArea/parentSurfaceArea)*leftCount 
                                  + (rightSurfaceArea/parentSurfaceArea)*rightCount );
                    else
                        SAH = this._tree._kT + this._tree._kI * ( (leftSurfaceArea/parentSurfaceArea)*leftCount 
                                  + (rightSurfaceArea/parentSurfaceArea)*rightCount)
                                  - this._tree._kO * (overlapSurfaceArea/parentSurfaceArea)*(rightCount+leftCount);

                    b = leftCount - rightCount;
                    if(b <= 1 && b >= 0 ) SAH *= this._tree._kB;

                    return SAH;
                },

                // returns best axis and planes to split sortedArraysOfNodes into left and right regions
                getBestSplit : function(sortedArraysOfNodes, AABB, totalWeight) { 
                    var parentSurfaceArea = AABB.getSurfaceArea(),
                        cheapestAxis = -1,
                        cheapestIndex = -1,
                        cheapestCost = Infinity, // Just some large value
                        cheapestLeftPlane = -1,
                        cheapestRightPlane = -1,
                        numberOfAxis = sortedArraysOfNodes.length, // Length of bounding box array
                        costOfTotalIntersection = 0,
                        totalCount = 0,
                        numberOfElements = 0,
                        currentLeftPlane = Math.NaN,
                        currentRightPlane = 0,
                        currentLeftCount = 0,
                        currentRightCount = 0,
                        currentCost = 0,
                        currentStart = 0,
                        currentEnd = 0,
                        leftAABB,
                        rightAABB,
                        elementArray,
                        element,
                        nextElement,
                        currentLeftWeight = 0,
                        currentRightWeight = 0,
                        cheapestLeftWeight = 0,
                        cheapestRightWeight = 0;

                    totalCount = sortedArraysOfNodes[0].length;
                    costOfTotalIntersection = this._tree._kI * totalCount;

                    while(numberOfAxis-->0){
                        leftAABB = AABB.clone();
                        rightAABB = AABB.clone();

                        elementArray = sortedArraysOfNodes[numberOfAxis];

                        currentLeftWeight = 0;
                        currentLeftCount = 0;
                        currentRightWeight = totalWeight;
                        currentRightCount = numberOfElements = elementArray.length;
                        //currentRightPlane = currentLeftPlane = elementArray[numberOfElements-1].a;
                        currentLeftPlane = Math.NaN;

                        element = elementArray[numberOfElements - 1];
                        while(numberOfElements-->1){
                            //move one element at a time to the left and find the score
                            nextElement = elementArray[numberOfElements - 1];
                            currentLeftCount++;
                            currentLeftWeight += element.w;
                            currentRightCount--;
                            currentRightWeight -= element.w;

                            currentEnd = element.i.max[numberOfAxis];
                            currentStart = nextElement.i.min[numberOfAxis];

                            currentLeftPlane = currentLeftPlane == Math.NaN ? currentEnd + Number.EPSILON : Math.max(currentLeftPlane, currentEnd + Number.EPSILON);
                            currentRightPlane = currentStart - Number.EPSILON;
                            leftAABB.max[numberOfAxis] = currentLeftPlane;
                            rightAABB.min[numberOfAxis] = currentRightPlane;

                            if(currentLeftCount < this._tree._minLeaf || currentRightCount < this._tree._minLeaf) {
                                currentCost = Math.NaN;
                            } else {
                                currentCost = this.calculateSplitCost(
                                    numberOfAxis,
                                    currentLeftPlane,
                                    currentRightPlane,
                                    currentLeftWeight,
                                    currentRightWeight,
                                    leftAABB,
                                    rightAABB,
                                    parentSurfaceArea
                                );
                            }

                            if(currentCost && (cheapestIndex + cheapestAxis < 0 || currentCost < cheapestCost)) {
                                cheapestAxis = numberOfAxis;
                                cheapestIndex = numberOfElements;
                                cheapestCost = currentCost;
                                cheapestLeftPlane = currentLeftPlane;
                                cheapestRightPlane = currentRightPlane;
                                cheapestLeftWeight = currentLeftWeight;
                                cheapestRightWeight = currentRightWeight;
                            }

                            element = nextElement;
                        }
                    }

                    if(cheapestIndex < 0 || (totalCount <= this._tree._maxLeaf && cheapestCost > costOfTotalIntersection)) return false;
                    return({
                        axis: cheapestAxis,
                        index: cheapestIndex,
                        left: cheapestLeftPlane,
                        right: cheapestRightPlane,
                        cost: cheapestCost,
                        leftWeight: cheapestLeftWeight,
                        rightWeight: cheapestRightWeight
                    });
                }
            };
        },
  
        _makeUnfinishedNode : function(boundingBox, sortedArraysOfNodes, totalWeight) {
            return {
                i: boundingBox,
                s: sortedArraysOfNodes,
                w: totalWeight
            };
        },

        _makeSplittingNode : function(leftPlane, rightPlane, leftNode, rightNode, axis, cost) {
            return {
                x: axis,
                u: leftPlane,
                v: rightPlane,
                l: leftNode,
                r: rightNode
            };
        },

        _makeLeafNode : function(boundingBox, elements) {
            return {
                //			i: boundingBox,
                o: elements
            };
        },

        _deleteLeafNode : function(node) {
            // this == node's parent
        },

        _buildNodeOrLeaf : function(AABB, sortedArraysOfNodes, localWeight, childBuilder) {
            var numberOfElements = sortedArraysOfNodes[0].length;

            if(numberOfElements <= this._minLeaf) return this._makeLeafNode(
                this.nodeHelpers.makeMBV(sortedArraysOfNodes[0]), sortedArraysOfNodes[0]);

            // scan across all sorrted-axises for a best fit
            var bestSplit = this.treeBuilder.getBestSplit(sortedArraysOfNodes, AABB, localWeight);

            // If it is cheaper to build a leaf, do so
            if(!bestSplit) return this._makeLeafNode(
                this.nodeHelpers.makeMBV(sortedArraysOfNodes[0]), sortedArraysOfNodes[0]);

            // Make each node's AABB
            var leftAABB = AABB.clone(),
                rightAABB = AABB.clone(),
                newArraysOfSortedNodes = this.nodeHelpers.splitSortedNodeArrays(
                    sortedArraysOfNodes,
                    bestSplit.axis,
                    bestSplit.index,
                    bestSplit.left,
                    bestSplit.right);

            leftAABB.max[bestSplit.axis] = bestSplit.left;
            rightAABB.min[bestSplit.axis] = bestSplit.right;

            // make and return a new node
            // call childBuilder twice for each node
            return this._makeSplittingNode(
                bestSplit.left,
                bestSplit.right,
                childBuilder.call(this, leftAABB, newArraysOfSortedNodes[0], bestSplit.leftWeight),
                childBuilder.call(this, rightAABB, newArraysOfSortedNodes[1], bestSplit.rightWeight),
                bestSplit.axis + 1,
                bestSplit.cost
            );
        },

        _recursiveBuild : function(AABB, sortedArraysOfNodes, localWeight) {
            return this._buildNodeOrLeaf(AABB, sortedArraysOfNodes, localWeight, this._recursiveBuild);
        },

        _incrementalBuild : function(unfinishedNode){
            var sortedArraysOfNodes = unfinishedNode.s,
                AABB = unfinishedNode.i,
                localWeight = unfinishedNode.w;

            return this._buildNodeOrLeaf(AABB, sortedArraysOfNodes, localWeight, this._makeUnfinishedNode);
        },

        buildFromArrayOfNodes : function(arrayOfNodes, deferredBuild){
            //make sorted lists of nodes. one list per axis sorted by min aabb
            var sortedArraysOfNodes = this.nodeHelpers.makeSortedArrays(arrayOfNodes),
                totalWeight = this.nodeHelpers.makeWeight(arrayOfNodes);

            this.i = this.nodeHelpers.makeMBV(arrayOfNodes);

            if(deferredBuild)
                this._T = this._makeUnfinishedNode(this.i, sortedArraysOfNodes, totalWeight);
            else
                this._T = this._recursiveBuild(this.i, sortedArraysOfNodes, totalWeight);
        },

        buildFromArrayOfElements : function(arrayOfElements, deferredBuild){
            this.buildFromArrayOfNodes(this.nodeHelpers.makeArrayOfNodes(arrayOfElements), deferredBuild);
        },

        buildFromArrayOfNodesAsync : function(arrayOfNodes, progressCallback, finishedCallback){
            var finishedElementCount = 0,
                totalElementsCount = arrayOfNodes.length,
                nodesTodo = [],
                doLeft = [],
                currentNode,
                wasLeft,
            //make sorted lists of nodes. one list per axis sorted by min aabb
                sortedArraysOfNodes = this.nodeHelpers.makeSortedArrays(arrayOfNodes),
                totalWeight = this.nodeHelpers.makeWeight(arrayOfNodes),
                thisTree = this;

            this.i = this.nodeHelpers.makeMBV(arrayOfNodes);

            // Make root..
            this._T = this._incrementalBuild(this._makeUnfinishedNode(this.i, sortedArraysOfNodes, totalWeight));
            if(this._T.o) return finishedCallback();
            nodesTodo.push(this._T);
            doLeft.push(true);

            if(progressCallback)
                progressCallback({phase: "Building acceleration structure...", percent: 0});

            process.nextTick(makeTree);

            function makeTree() {
                var startTime = Date.now();
                while(nodesTodo.length) {
                    if(Date.now() - startTime > 1000) {
                        if(progressCallback)
                            progressCallback({percent: finishedElementCount / totalElementsCount * 100});
                        return process.nextTick(makeTree);
                    }

                    currentNode = nodesTodo.pop();
                    wasLeft = doLeft.pop();

                    if(!wasLeft) {
                        currentNode.r = thisTree._incrementalBuild(currentNode.r);
                        if(currentNode.r.o) {
                            finishedElementCount += currentNode.r.o.length;
                        } else {
                            nodesTodo.push(currentNode.r);
                            doLeft.push(true);
                        }
                    } else {
                        currentNode.l = thisTree._incrementalBuild(currentNode.l);
                        nodesTodo.push(currentNode);
                        doLeft.push(false);
                        if(currentNode.l.o) {
                            finishedElementCount += currentNode.l.o.length;
                        } else {
                            nodesTodo.push(currentNode.l);
                            doLeft.push(true);
                        }
                    }
                }
                return finishedCallback(null, thisTree);
            }
        },

        buildFromArrayOfElementsAsync : function(arrayOfElements, progressCallback, finishedCallback){
            var thisTree = this;
            this.nodeHelpers.makeArrayOfNodesAsync(arrayOfElements, progressCallback, function(arrayOfNodes){
                thisTree.buildFromArrayOfNodesAsync(arrayOfNodes, progressCallback, finishedCallback);
            });
        },

        // tree.each (recursive)
        // go over each node and leaf in the tree (depth-first) and execute the provided
        // callback functions.
        // Very useful for drawing the tree, counting nodes, ect.
        each : function(nodeCallback, leafCallback) {
            function iterate(node, currentAABB, depth){
                var axis,
                    leftAABB,
                    rightAABB, retVal;

                if(node.x){
                    axis = node.x - 1;

                    leftAABB = currentAABB.clone();
                    leftAABB.max[axis] = node.u;

                    rightAABB = currentAABB.clone();
                    rightAABB.min[axis] = node.v;

                    if(nodeCallback) retVal = nodeCallback(node, leftAABB, rightAABB, depth);
                    if(!retVal) return;

                    iterate(node.l, leftAABB, depth+1);
                    iterate(node.r, rightAABB, depth+1);
                } else if(node.o) {
                    if(leafCallback) leafCallback(node, currentAABB, depth);
                }
            }

            iterate(this._T, this.i, 0);
        },

        // tree.intersectStep 
        // intersect the tree with ray returning the earliest element that hits the ray
        // returns information to a callback and provides a next function to continue 
        // traversal
        //
        // Based on Havran's TA-B -
        // Modified for BIH with several new cases:
        //  - I1 (ray enters between planes, exits in left node)
        //  - I2 (ray enters between planes, exits in right node)
        //  - I3 (ray enters between planes, exits between planes)
        //  - B1 (Ray enters in both planes, exits both)
        //  - B2 (Ray enters in both planes, exits left)
        //  - B3 (Ray enters in both planes, exits right)
        //
        // Modified for incremental(on-demand?) tree building
        //
        // stepCallback(node, rs, currentAABB, wasLeft, depth, next)
        intersectStep : function(ray, intersectInfo, stepCallback, finishedCallback) {
            var parentStack = [], // Contains the nodes that are parents of the hit nodes
                rayStack = [], // Contains the ray-segment for the current sub-tree
                depthStack = [], // Just for depth determining. debugging stuff.
                directionStack = [], // Which was the last direction taken - true = left
                aabbStack = [],
                currentAABB = this.i.clone(),
                tempAABB,
                lastDirectionWasLeft = true,
                rayIntervals = ray.toIntervals(),
                majorAxis = ray.getMajorAxis(),
                node = null,
                parentNode = null,
                rs = null,
                axis = null,
                newRS = null,
                bestSegment = this.segmentHelpers.cloneSegment(rayIntervals),
                leafElementCount,
                leafElement = null,
                noDebug = true,
                debug = null,
                thisTree = this;

            if(intersectInfo.debug) {
                noDebug = false;
                debug = intersectInfo.debug;
                depthStack = [];
            }

            // Test ray AABB first
            rs = this.i.intersectWithSegment(rayIntervals);

            // If there are no elements or the ray-AABB test failed, don't bother traversing
            if (rs && this._T !== null) {
                node = this._T;
            } else {
                if(finishedCallback) return finishedCallback(intersectInfo);
                return;
            }

            stepCallback(node, rs, currentAABB, null, 0, step);

            function step() {
                while( (node !== null) || parentStack.length > 0) {
                    if(!noDebug) debug.currDepth++;

                    if(node === null) {
                        rs = rayStack.pop(); // Depth-First Descent
                        parentNode = parentStack.pop();
                        lastDirectionWasLeft = directionStack.pop();
                        currentAABB = aabbStack.pop();
                        if(!noDebug) {
                            debug.depth = Math.max(debug.depth, debug.currDepth);
                            debug.currDepth = depthStack.pop();
                        }


                        if(lastDirectionWasLeft)
                            node = parentNode.l;
                        else
                            node = parentNode.r;
                    }

                    // Check to see if this node is still reachable
                    if(intersectInfo.isHit) {
                        if(!thisTree.segmentHelpers.trimSegmentInPlace(bestSegment, rs, majorAxis)) {
                            node = null;
                            continue;
                        }
                    }

                    if(!noDebug) debug.costT++;

                    if(node.s) { // An unfinished node!
                        node = thisTree._incrementalBuild(node);
                        if(parentNode == null) {
                            thisTree._T = node;
                        } else {
                            if(lastDirectionWasLeft)
                                parentNode.l = node;
                            else
                                parentNode.r = node;
                        }
                    }

                    // Below is not an "else if" because we want to continue traversal during an incremental build
                    if (node.x) { // A node!
                        axis = node.x - 1; // Axis: 1 = x, 2 = y, 3 = z ...
                        // Cases where entry point is between *both* splitting planes
                        if (rs[axis].a < node.v && rs[axis].a > node.u) {
                            if (rs[axis].b <= node.u) { // Is the exit point inside the left node?
                                /* case I1 */
                                // The ray enters the volume between the planes so
                                // we need to clip the ray start for this case
                                thisTree.segmentHelpers.clipSegmentStartInPlace(rs, axis, node.u);
                                parentNode = node;
                                lastDirectionWasLeft = true;
                                currentAABB.max[axis] = node.u;
                                node = node.l;
                            } else if (rs[axis].b >= node.v) { // Is the exit point inside the right node?
                                /* case I2 */
                                // The ray enters the volume between the planes so
                                // we need to clip the ray start for this case
                                thisTree.segmentHelpers.clipSegmentStartInPlace(rs, axis, node.v);
                                parentNode = node;
                                lastDirectionWasLeft = false;
                                currentAABB.min[axis] = node.v;
                                node = node.r;
                            } // If start is between both planes,
                                // the end point CAN NOT be in BOTH nodes - it is unpossible
                            else {
                                node = null;
                            }
                        } else if (rs[axis].a <= node.u) { // Starts in left node
                            if (rs[axis].a >= node.v) { // Also in right node!
                                // If we exit and are no longer in the right node, we must clip the ray
                                if (rs[axis].b < node.v) {
                                    newRS = thisTree.segmentHelpers.clipSegmentEnd(rs, axis, node.v);
                                } else {
                                    newRS = thisTree.segmentHelpers.cloneSegment(rs);
                                }
                                // This will be popped later, so right = far node
                                rayStack.push(newRS);
                                parentStack.push(node);
                                directionStack.push(false);
                                tempAABB = currentAABB.clone();
                                tempAABB.min[axis] = node.v;
                                aabbStack.push(tempAABB);
                                if(!noDebug) depthStack.push(debug.currDepth);
									
                                // If we exit and are no longer in the left node, we must clip the ray
                                if (rs[axis].b > node.u) {
                                    thisTree.segmentHelpers.clipSegmentEndInPlace(rs, axis, node.u);
                                }
                                // This will be popped first, so left = near node
                                parentNode = node;
                                lastDirectionWasLeft = true;
                                currentAABB.max[axis] = node.u;
                                node = node.l;
                            } else if (rs[axis].b < node.v) { // We are exiting before the right plane
                                if (rs[axis].b <= node.u) {
                                    // We are exiting before the left plane
                                    /* cases N1,N2,N3,P5,Z2,Z3 */
                                    parentNode = node;
                                    lastDirectionWasLeft = true;
                                    currentAABB.max[axis] = node.u;
                                    node = node.l;
                                } else {
                                    // The ray exits the volume between the planes so
                                    // we need to clip the ray end for this case
                                    thisTree.segmentHelpers.clipSegmentEndInPlace(rs, axis, node.u);
                                    parentNode = node;
                                    lastDirectionWasLeft = true;
                                    currentAABB.max[axis] = node.u;
                                    node = node.l;
                                }
                            } else { // The ray exits on the far side of the right plane
                                /* case N4 */
                                // This will be popped later, so right = far node
                                newRS = thisTree.segmentHelpers.clipSegmentStart(rs, axis, node.v);

                                rayStack.push(newRS);
                                parentStack.push(node);
                                directionStack.push(false);
                                tempAABB = currentAABB.clone();
                                tempAABB.min[axis] = node.v;
                                aabbStack.push(tempAABB);
                                if(!noDebug) depthStack.push(debug.currDepth);

                                // This will be popped first, so left = near node
                                thisTree.segmentHelpers.clipSegmentEndInPlace(rs, axis, node.u);
                                parentNode = node;
                                lastDirectionWasLeft = true;
                                currentAABB.max[axis] = node.u;
                                node = node.l;
                            }
                        } else if (rs[axis].a >= node.v) { // Starts in right node
                            if (rs[axis].b > node.u) { // Ray exits before the left plane
                                if (rs[axis].b >= node.v) { // Ray exits before the right plane
                                    /* cases P1,P2,P3,N5,Z1 */
                                    parentNode = node;
                                    lastDirectionWasLeft = false;
                                    currentAABB.min[axis] = node.v;
                                    node = node.r;
                                } else { /* cases P1,P2,P3,N5,Z1 */
                                    // we need to clip the ray end for this case
                                    thisTree.segmentHelpers.clipSegmentEndInPlace(rs, axis, node.v);
                                    parentNode = node;
                                    lastDirectionWasLeft = false;
                                    currentAABB.min[axis] = node.v;
                                    node = node.r;
                                }
                            } else { // Ray hits both planes
                                /* case P4 */
                                // This will be popped later, so left = far node
                                newRS = thisTree.segmentHelpers.clipSegmentStart(rs, axis, node.u);
                                rayStack.push(newRS);
                                parentStack.push(node);
                                directionStack.push(true);
                                tempAABB = currentAABB.clone();
                                tempAABB.max[axis] = node.u;
                                aabbStack.push(tempAABB);
                                if(!noDebug) depthStack.push(debug.currDepth);

                                // This will be popped first, so right = near node
                                thisTree.segmentHelpers.clipSegmentEndInPlace(rs, axis, node.v);
                                parentNode = node;
                                lastDirectionWasLeft = false;
                                currentAABB.min[axis] = node.v;
                                node = node.r;
                            }
                        } else {
                            node = null;
                        }
                    }

                    if (node && node.x) return stepCallback(node, rs, currentAABB, lastDirectionWasLeft, debug.currDepth, step);

                    if (node && node.o) { // A leaf!!
                        leafElementCount = node.o.length;
                        while(leafElementCount-->0) {
                            if(!noDebug) debug.costI++;

                            leafElement = node.o[leafElementCount];
                            leafElement.iFn.call(leafElement.o, ray, intersectInfo);

                            if(intersectInfo.isHit) {
                                thisTree.segmentHelpers.setBestSegment(bestSegment, intersectInfo.position);
                            }
                        }// end while each element

                        // Causes the loop to pop far-node candidates from the stack
                        stepCallback(node, rs, currentAABB, lastDirectionWasLeft, debug.currDepth, step);
                        node = null;
                    }
                }// end of main while()
                if(!noDebug) debug.depth = Math.max(debug.depth, debug.currDepth);
                if(finishedCallback) finishedCallback(intersectInfo);
            }
        },

        // tree.intersect (non-recursive)
        // intersect the tree with ray returning the earliest element that hits the ray
        //
        // Based on Havran's TA-B -
        // Modified for BIH with several new cases:
        //  - I1 (ray enters between planes, exits in left node)
        //  - I2 (ray enters between planes, exits in right node)
        //  - I3 (ray enters between planes, exits between planes)
        //  - B1 (Ray enters in both planes, exits both)
        //  - B2 (Ray enters in both planes, exits left)
        //  - B3 (Ray enters in both planes, exits right)
        //
        // Modified for incremental(on-demand?) tree building
        intersect : function(ray, intersectInfo) {
            var parentStack = [], // Contains the nodes that are parents of the hit nodes
                rayStack = [], // Contains the ray-segment for the current sub-tree
                depthStack = [], // Just for depth determining. debugging stuff.
                directionStack = [], // Which was the last direction taken - true = left
                lastDirectionWasLeft = true,
                rayIntervals = ray.toIntervals(),
                majorAxis = ray.getMajorAxis(),
                node = null,
                parentNode = null,
                rs = null,
                axis = null,
                newRS = null,
                bestSegment = this.segmentHelpers.cloneSegment(rayIntervals),
                leafElementCount,
                leafElement = null,
                noDebug = true,
                debug = null;

            if(intersectInfo.debug) {
                noDebug = false;
                debug = intersectInfo.debug;
                depthStack = [];
            }

            // Test ray AABB first
            rs = this.i.intersectWithSegment(rayIntervals);

            // If there are no elements or the ray-AABB test failed, don't bother traversing
            if (rs && this._T !== null) {
                node = this._T;
            } else {
                return;
            }

            while( (node !== null) || parentStack.length > 0) {
                if(!noDebug) debug.currDepth++;

                if(node === null) {
                    rs = rayStack.pop(); // Depth-First Descent
                    parentNode = parentStack.pop();
                    lastDirectionWasLeft = directionStack.pop();
                    if(!noDebug) {
                        debug.depth = Math.max(debug.depth, debug.currDepth);
                        debug.currDepth = depthStack.pop();
                    }


                    if(lastDirectionWasLeft)
                        node = parentNode.l;
                    else
                        node = parentNode.r;
                }

                // Check to see if this node is still reachable
                if(intersectInfo.isHit) {
                    if(!this.segmentHelpers.trimSegmentInPlace(bestSegment, rs, majorAxis)) {
                        node = null;
                        continue;
                    }
                }
                if(node.s) { // An unfinished node!
                    node = this._incrementalBuild(node);
                    if(parentNode == null) {
                        this._T = node;
                    } else {
                        if(lastDirectionWasLeft)
                            parentNode.l = node;
                        else
                            parentNode.r = node;
                    }
                }

                // Below is not an "else if" because we want to continue traversal during an incremental build
                if (node.x) { // A node!
                    axis = node.x - 1; // Axis: 1 = x, 2 = y, 3 = z ...
                    // Cases where entry point is between *both* splitting planes
                    if (rs[axis].a < node.v && rs[axis].a > node.u) {
                        if (rs[axis].b <= node.u) { // Is the exit point inside the left node?
                            /* case I1 */
                            // The ray enters the volume between the planes so
                            // we need to clip the ray start for this case
                            this.segmentHelpers.clipSegmentStartInPlace(rs, axis, node.u);
                            parentNode = node;
                            lastDirectionWasLeft = true;
                            node = node.l;
                        } else if (rs[axis].b >= node.v) { // Is the exit point inside the right node?
                            /* case I2 */
                            // The ray enters the volume between the planes so
                            // we need to clip the ray start for this case
                            this.segmentHelpers.clipSegmentStartInPlace(rs, axis, node.v);
                            parentNode = node;
                            lastDirectionWasLeft = false;
                            node = node.r;
                        } // If start is between both planes,
                            // the end point CAN NOT be in BOTH nodes - it is unpossible
                        else {
                            node = null;
                        }
                    } else if (rs[axis].a <= node.u) { // Starts in left node
                        if (rs[axis].a >= node.v) { // Also in right node!
                            // If we exit and are no longer in the right node, we must clip the ray
                            if (rs[axis].b < node.v) {
                                newRS = this.segmentHelpers.clipSegmentEnd(rs, axis, node.v);
                            } else {
                                newRS = this.segmentHelpers.cloneSegment(rs);
                            }
                            // This will be popped later, so right = far node
                            rayStack.push(newRS);
                            parentStack.push(node);
                            directionStack.push(false);
                            if(!noDebug) depthStack.push(debug.currDepth);
								
                            // If we exit and are no longer in the left node, we must clip the ray
                            if (rs[axis].b > node.u) {
                                this.segmentHelpers.clipSegmentEndInPlace(rs, axis, node.u);
                            }
                            // This will be popped first, so left = near node
                            parentNode = node;
                            lastDirectionWasLeft = true;
                            node = node.l;
                        } else if (rs[axis].b < node.v) { // We are exiting before the right plane
                            if (rs[axis].b <= node.u) {
                                // We are exiting before the left plane
                                /* cases N1,N2,N3,P5,Z2,Z3 */
                                parentNode = node;
                                lastDirectionWasLeft = true;
                                node = node.l;
                            } else {
                                // The ray exits the volume between the planes so
                                // we need to clip the ray end for this case
                                this.segmentHelpers.clipSegmentEndInPlace(rs, axis, node.u);
                                parentNode = node;
                                lastDirectionWasLeft = true;
                                node = node.l;
                            }
                        } else { // The ray exits on the far side of the right plane
                            /* case N4 */
                            // This will be popped later, so right = far node
                            newRS = this.segmentHelpers.clipSegmentStart(rs, axis, node.v);

                            rayStack.push(newRS);
                            parentStack.push(node);
                            directionStack.push(false);
                            if(!noDebug) depthStack.push(debug.currDepth);

                            // This will be popped first, so left = near node
                            this.segmentHelpers.clipSegmentEndInPlace(rs, axis, node.u);
                            parentNode = node;
                            lastDirectionWasLeft = true;
                            node = node.l;
                        }
                    } else if (rs[axis].a >= node.v) { // Starts in right node
                        if (rs[axis].b > node.u) { // Ray exits before the left plane
                            if (rs[axis].b >= node.v) { // Ray exits before the right plane
                                /* cases P1,P2,P3,N5,Z1 */
                                parentNode = node;
                                lastDirectionWasLeft = false;
                                node = node.r;
                            } else { /* cases P1,P2,P3,N5,Z1 */
                                // we need to clip the ray end for this case
                                this.segmentHelpers.clipSegmentEndInPlace(rs, axis, node.v);
                                parentNode = node;
                                lastDirectionWasLeft = false;
                                node = node.r;
                            }
                        } else { // Ray hits both planes
                            /* case P4 */
                            // This will be popped later, so left = far node
                            newRS = this.segmentHelpers.clipSegmentStart(rs, axis, node.u);
                            rayStack.push(newRS);
                            parentStack.push(node);
                            directionStack.push(true);
                            if(!noDebug) depthStack.push(debug.currDepth);

                            // This will be popped first, so right = near node
                            this.segmentHelpers.clipSegmentEndInPlace(rs, axis, node.v);
                            parentNode = node;
                            lastDirectionWasLeft = false;
                            node = node.r;
                        }
                    } else {
                        node = null;
                    }
                }

                if (node && node.o) { // A leaf!!
                    leafElementCount = node.o.length;
                    while(leafElementCount-->0) {
                        leafElement = node.o[leafElementCount];
                        leafElement.iFn.call(leafElement.o, ray, intersectInfo);
                        if(intersectInfo.isHit) {
                            this.segmentHelpers.setBestSegment(bestSegment, intersectInfo.position);
                            return node.o;
                        }
                    }// end while each element

                    // Causes the loop to pop far-node candidates from the stack
                    node = null;
                }
            }// end of main while()
            if (!noDebug) debug.depth = Math.max(debug.depth, debug.currDepth);
        },

        overlaps : function (testAABB, returnArray) {
            return this._search(testAABB, returnArray, "contains", "oFn");
        },

        contains : function (testAABB, returnArray) {
            return this._search(testAABB, returnArray, "contains", "cdFn");
        },

        contained : function (testAABB, returnArray) {
            return this._search(testAABB, returnArray, "contained", "csFn", true);
        },

        // tree.overlaps (non-recursive) -
        // Search a region defined by testAABB and return an array of all elements
        // that overlap this region.
        //
        // Posible node-intervals cases during traversal:
        //   B0 - Interval starts in left, ends in right;
        //   L0 - Interval starts in left, ends in left;
        //   L1 - Interval starts in left, ends in gap;
        //   R0 - Interval starts in right, ends in right;
        //   R1 - Interval starts in gap, ends in right;
        //   G0 - Interval starts in gap, ends in gap;
        // Modified for incremental(on-demand?) tree building
        _search : function(testAABB, returnArray, aabbTestFunction, leafTestFunction, useAlteredLogic) {
            var parentStack = [], // Contains the nodes that are parents of the hit nodes
                directionStack = [], // Which was the last direction taken - true = left
                lastDirectionWasLeft = true,
                returnArray = returnArray || [],
                node = null,
                parentNode = null,
                axis = null,
                leafElementCount,
                leafElement = null,
                overlaps, contained,
                alteredLogic = !!useAlteredLogic;

            if(this._T === null) return returnArray;

            overlaps = testAABB.overlaps(this.i);
            contained = testAABB[aabbTestFunction](this.i);

            if(overlaps) {
                if(!alteredLogic){
                    node = this._T;
                } else {
                    if(contained) node = this._T;
                    else return returnArray;
                }
            } else {
                return returnArray;
            }

            while( (node != null) || parentStack.length > 0) {

                if(node == null) {
                    parentNode = parentStack.pop();
                    lastDirectionWasLeft = directionStack.pop();

                    if(lastDirectionWasLeft)
                        node = parentNode.l;
                    else
                        node = parentNode.r;
                }

                if(node.s) { // An unfinished node!
                    node = this._incrementalBuild(node);
                    if(parentNode == null) {
                        this._T = node;
                    } else {
                        if(lastDirectionWasLeft)
                            parentNode.l = node;
                        else
                            parentNode.r = node;
                    }
                }

                // Below is not an "else if" because we want to continue traversal during 
                // an incremental build
                if (node.x) { // A node!
                    axis = node.x - 1; // Axis: 1 = x, 2 = y, 3 = z ... so on
                    // Cases where entry point is between *both* splitting planes
						
                    if(testAABB.min[axis] <= node.u) { // Starts in left node
                        if(testAABB.max[axis] >= node.v) { // B0 Ends in right - Both nodes
                            if(!alteredLogic || testAABB.min[axis] >= node.v) {
                                parentStack.push(node);
                                directionStack.push(false);
                            }
                        } // L0, L1, Left node
                        if(!alteredLogic || testAABB.max[axis] <= node.u) {
                            parentNode = node;
                            lastDirectionWasLeft = true;
                            node = node.l;
                        } else {
                            node = null;
                        }
                    } else if(testAABB.max[axis] >= node.v) { // Ends in right
                        // R0, R1 - only Right node
                        if(!alteredLogic || testAABB.min[axis] >= node.v) {
                            parentNode = node;
                            lastDirectionWasLeft = false;
                            node = node.r;
                        } else {
                            node = null;
                        }
                    } else { // G0 - no nodes
                        node = null;
                    }
                }

                if (node && node.o) { // A Leaf !!
                    leafElementCount = node.o.length;
                    while(leafElementCount-->0) {
                        leafElement = node.o[leafElementCount];
                        leafElement[leafTestFunction].call(leafElement.o, testAABB, returnArray);
                    }
                    node = null;
                }
            }// end of main while()
            return (returnArray);
        },

        toJSON : function() {
            return JSON.stringify(this._T);
        }
    };

