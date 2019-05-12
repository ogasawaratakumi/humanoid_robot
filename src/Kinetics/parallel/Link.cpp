#include "Link.h"

extern "C" void SetJointInfo(struct Link *link)
{
	for(int i=0;i<LINK_NUM;i++)
	{
		link[i].joint_name = joint_name[i];
		link[i].parent = LINK_CONNECT[i].parent;
		link[i].child  = LINK_CONNECT[i].child;
		link[i].sister = LINK_CONNECT[i].sister;
		for(int j=0;j<3;j++)
		{
			link[i].a(j) = LinkAxis[i][j];
			link[i].b(j) = LinkPos[i][j]/100.0;

		}
	}
}
