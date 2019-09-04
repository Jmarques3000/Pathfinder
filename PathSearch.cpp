#include "PathSearch.h"

namespace fullsail_ai {
	namespace algorithms {

		PathSearch::PathSearch()
		{
		}

		PathSearch::~PathSearch()
		{
		}

		float PathSearch::estimate(SearchNode* start, SearchNode* finish)
		{
			//estimate how far end is from start
			float estimate = sqrt(pow((finish->current->getXCoordinate() - start->current->getXCoordinate()), 2) + pow((finish->current->getYCoordinate() - start->current->getYCoordinate()), 2));
			return estimate;
		}

		void PathSearch::initialize(TileMap* _tileMap)
		{

			map = _tileMap;
			int rows = _tileMap->getRowCount();
			int columns = _tileMap->getColumnCount();
			for (size_t j = 0; j < columns; j++)
			{
				for (size_t i = 0; i < rows; i++)
				{
					if (_tileMap->getTile(i, j)->getWeight() != 0)
					{
						SearchNode *here = new SearchNode;
						here->current = _tileMap->getTile(i, j);
						search[_tileMap->getTile(i, j)] = here;
					}
				}
			}

			for (size_t j = 0; j < columns; j++)
			{
				for (size_t i = 0; i < rows; i++)
				{
					if (search[_tileMap->getTile(i, j)])
					{
						if (i % 2 == 0)
						{
							if (search[_tileMap->getTile(i - 1, j - 1)])
							{
								search[_tileMap->getTile(i, j)]->neighbors.push_back(search[_tileMap->getTile(i - 1, j - 1)]);
							}
							if (search[_tileMap->getTile(i - 1, j)])
							{
								search[_tileMap->getTile(i, j)]->neighbors.push_back(search[_tileMap->getTile(i - 1, j)]);
							}
							if (search[_tileMap->getTile(i, j - 1)])
							{
								search[_tileMap->getTile(i, j)]->neighbors.push_back(search[_tileMap->getTile(i, j - 1)]);
							}
							if (search[_tileMap->getTile(i, j + 1)])
							{
								search[_tileMap->getTile(i, j)]->neighbors.push_back(search[_tileMap->getTile(i, j + 1)]);
							}
							if (search[_tileMap->getTile(i + 1, j - 1)])
							{
								search[_tileMap->getTile(i, j)]->neighbors.push_back(search[_tileMap->getTile(i + 1, j - 1)]);
							}
							if (search[_tileMap->getTile(i + 1, j)])
							{
								search[_tileMap->getTile(i, j)]->neighbors.push_back(search[_tileMap->getTile(i + 1, j)]);
							}
						}
						else
						{
							if (search[_tileMap->getTile(i - 1, j)])
							{
								search[_tileMap->getTile(i, j)]->neighbors.push_back(search[_tileMap->getTile(i - 1, j)]);
							}
							if (search[_tileMap->getTile(i - 1, j + 1)])
							{
								search[_tileMap->getTile(i, j)]->neighbors.push_back(search[_tileMap->getTile(i - 1, j + 1)]);
							}
							if (search[_tileMap->getTile(i, j - 1)])
							{
								search[_tileMap->getTile(i, j)]->neighbors.push_back(search[_tileMap->getTile(i, j - 1)]);
							}
							if (search[_tileMap->getTile(i, j + 1)])
							{
								search[_tileMap->getTile(i, j)]->neighbors.push_back(search[_tileMap->getTile(i, j + 1)]);
							}
							if (search[_tileMap->getTile(i + 1, j)])
							{
								search[_tileMap->getTile(i, j)]->neighbors.push_back(search[_tileMap->getTile(i + 1, j)]);
							}
							if (search[_tileMap->getTile(i + 1, j + 1)])
							{
								search[_tileMap->getTile(i, j)]->neighbors.push_back(search[_tileMap->getTile(i + 1, j + 1)]);
							}
						}
					}
				}
			}
		}

		void PathSearch::enter(int startRow, int startColumn, int goalRow, int goalColumn)
		{
			trueend = false;
			PlannerNode* started = new PlannerNode;
			started->parent = nullptr;
			queue.push(started);
			planmap[search[map->getTile(startRow, startColumn)]] = started;
			started->vertex = search[map->getTile(startRow, startColumn)];
			end = search[map->getTile(goalRow, goalColumn)];
			started->givencost = 0;
			nodetonext = estimate(started->vertex, started->vertex->neighbors[0]);
			started->heuristiccost = estimate(started->vertex, end);
			started->totalcost = started->givencost + started->heuristiccost * started->vertex->current->getWeight();
		}

		void PathSearch::update(long timeslice)
		{
			while (!queue.empty())
			{
				PlannerNode* current = queue.front();
				current->heuristiccost = estimate(current->vertex, end);
				queue.pop();
				if (current->vertex == end)
				{
					makeSolution();
					trueend = true;
					return;
				}
				for (size_t i = 0; i < current->vertex->neighbors.size(); i++)
				{
					SearchNode *succesor = current->vertex->neighbors[i];
					float tempgivencost = current->givencost + nodetonext * succesor->current->getWeight();
					if (planmap[succesor] != nullptr)
					{
						if (tempgivencost < planmap[succesor]->givencost)
						{
							PlannerNode *succersorNode = planmap[succesor];
							queue.remove(succersorNode);
							succersorNode->vertex = succesor;
							succersorNode->givencost = tempgivencost;
							succersorNode->totalcost = succersorNode->givencost + succersorNode->heuristiccost * heuristicweight;
							succersorNode->parent = current;
							queue.push(succersorNode);
						}
					}
					else
					{
						PlannerNode *succersorNode = new PlannerNode;
						succersorNode->vertex = succesor;
						succersorNode->vertex->current->setFill(0xFF0000FF);
						succersorNode->parent = current;
						succersorNode->givencost = tempgivencost;
						succersorNode->heuristiccost = estimate(succesor, end);
						succersorNode->totalcost = succersorNode->givencost + succersorNode->heuristiccost * heuristicweight;
						planmap[succesor] = succersorNode;
						queue.push(succersorNode);
					}
				}
				if (timeslice == 0)
					return;
			}

		}

		void PathSearch::exit()
		{
			//map->reset();
			planmap.clear();
			//search.clear();
			while (!queue.empty())
			{
				queue.pop();
			}
			end = nullptr;
			temp.clear();
		}

		void PathSearch::shutdown()
		{
			search.clear();
		}

		bool PathSearch::isDone() const
		{
			if (trueend)
			{
			return true;
			}
			return false;
		}

		void PathSearch::makeSolution()
		{
			PlannerNode* backtrace;
			backtrace = planmap.at(end);
			while (backtrace->parent != nullptr)
			{
				backtrace->vertex->current->addLineTo(backtrace->parent->vertex->current, 0xFFFF0000);
				temp.push_back(backtrace->vertex->current);
				backtrace = backtrace->parent;
			}
			temp.push_back(backtrace->vertex->current);
		}

		std::vector<Tile const*> const PathSearch::getSolution() const
		{
			return temp;
		}
	}
}  // namespace fullsail_ai::algorithms

