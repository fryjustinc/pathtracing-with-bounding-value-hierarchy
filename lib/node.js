(function (root, factory) {
		"use strict";

		if (typeof exports === 'object') {
			module.exports = factory();
		} else if (typeof define === 'function' && define.amd) {
			define(factory);
		} else {
			if(!root.BxH) root.BxH = {};
			root.BxH.NodeHelpers = factory();
		}
	}(this, function () {
		"use strict";

		var cachedHelpers = {};

		function makeNodeFromElement(element) {
		    var aabb,
			    weight,
			    intersect,
			    overlaps,
			    contains,
			    contained,
                material;
		    material = element.material;

		    if("getAABB" in element) aabb = element.aabb;
		    else aabb = element.aabb;
		    material = element.material;

			if("getWeight" in element) weight = element.getWeight();
			else weight = element.weight;

			if("getIntersectName" in element) intersect = element.getIntersectFunction();
			else intersect =  element.intersect;

			if("getOverlapsName" in element) overlaps = element.getOverlapsFunction();
			else overlaps =  element.overlaps;

			if("getContainsName" in element) contains = element.getContainsFunction();
			else contains =  element.contains;

			if("getContainedName" in element) contained = element.getContainedFunction();
			else contained =  element.contained;

			return {
				i: aabb,
				w: weight,
				o: element,
				iFn: intersect,
				oFn: overlaps,
				csFn: contains,
				cdFn: contained
			};
		};



		return generateNodeHelpers;
}));