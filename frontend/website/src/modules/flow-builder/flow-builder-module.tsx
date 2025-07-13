import { Background, type Connection, type Edge, type EdgeTypes, type Node, type NodeChange, ReactFlow, addEdge, useEdgesState, useNodesState, useReactFlow } from "@xyflow/react";
import { nanoid } from "nanoid";
import { useCallback } from "react";

import CustomControls from "~/modules/flow-builder/components/controls/custom-controls";
import CustomDeletableEdge from "~/modules/flow-builder/components/edges/custom-deletable-edge";
import { defaultEdges, defaultNodes } from "~/modules/flow-builder/constants/default-nodes-edges";
import { useDeleteKeyCode } from "~/modules/flow-builder/hooks/use-delete-key-code";
import { useDragDropFlowBuilder } from "~/modules/flow-builder/hooks/use-drag-drop-flow-builder";
import { useIsValidConnection } from "~/modules/flow-builder/hooks/use-is-valid-connection";
import { useNodeAutoAdjust } from "~/modules/flow-builder/hooks/use-node-auto-adjust";
import { useOnNodesDelete } from "~/modules/flow-builder/hooks/use-on-nodes-delete";
import { NODE_TYPES } from "~/modules/nodes";
import { useApplicationState } from "~/stores/application-state";

const edgeTypes: EdgeTypes = {
    deletable: CustomDeletableEdge,
};

export function FlowBuilderModule() {
    const [isMobileView] = useApplicationState(s => [s.view.mobile]);

    const [nodes, _, onNodesChange] = useNodesState<Node>(defaultNodes);
    const [edges, setEdges, onEdgesChange] = useEdgesState<Edge>(defaultEdges);

    const { getNodes } = useReactFlow();

    const deleteKeyCode = useDeleteKeyCode();
    const onNodesDelete = useOnNodesDelete(nodes);

    const autoAdjustNode = useNodeAutoAdjust();

    const [onDragOver, onDrop] = useDragDropFlowBuilder();
    const isValidConnection = useIsValidConnection(nodes, edges);

    const onConnect = useCallback(
        (connection: Connection) => {
            const edge = { ...connection, id: nanoid(), type: "deletable" } as Edge;
            setEdges(edges => addEdge(edge, edges));
        },
        [setEdges],
    );

    const handleAutoAdjustNodeAfterNodeMeasured = useCallback(
        (id: string) => {
            setTimeout(() => {
                const node = getNodes().find(n => n.id === id);
                if (!node) { return; }

                if (node.measured === undefined) {
                    handleAutoAdjustNodeAfterNodeMeasured(id);
                    return;
                }

                autoAdjustNode(node);
            });
        },
        [autoAdjustNode, getNodes],
    );

    const handleNodesChange = useCallback(
        (changes: NodeChange[]) => {
            onNodesChange(changes);

            changes.forEach((change) => {
                if (change.type === "dimensions") {
                    const node = getNodes().find(n => n.id === change.id);
                    if (node) {
                        autoAdjustNode(node);
                    }
                }

                if (change.type === "add") {
                    handleAutoAdjustNodeAfterNodeMeasured(change.item.id);
                }
            });
        },
        [autoAdjustNode, getNodes, handleAutoAdjustNodeAfterNodeMeasured, onNodesChange],
    );

    return (
        <ReactFlow
            proOptions={{ hideAttribution: true }}
            nodeTypes={NODE_TYPES}
            onInit={({ fitView }) => fitView().then()}
            nodes={nodes}
            onNodesChange={handleNodesChange}
            edgeTypes={edgeTypes}
            edges={edges}
            onEdgesChange={onEdgesChange}
            onConnect={onConnect}
            onDrop={onDrop}
            onDragOver={onDragOver}
            onNodeDragStop={(_, node) => { autoAdjustNode(node); }}
            onNodesDelete={onNodesDelete}
            isValidConnection={isValidConnection}
            multiSelectionKeyCode={null}
            deleteKeyCode={deleteKeyCode}
            snapGrid={[16, 16]}
            snapToGrid
            fitView
        >
            <Background color={isMobileView ? "rgba(255,255,255,0.5)" : "rgba(255,255,255,0.25)"} gap={32} />
            <CustomControls />
        </ReactFlow>
    );
}
