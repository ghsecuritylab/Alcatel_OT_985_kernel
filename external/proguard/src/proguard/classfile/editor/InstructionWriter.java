
package proguard.classfile.editor;

import proguard.classfile.*;
import proguard.classfile.attribute.CodeAttribute;
import proguard.classfile.attribute.visitor.AttributeVisitor;
import proguard.classfile.instruction.*;
import proguard.classfile.instruction.visitor.InstructionVisitor;
import proguard.classfile.util.SimplifiedVisitor;

public class InstructionWriter
extends      SimplifiedVisitor
implements   InstructionVisitor,
             AttributeVisitor
{
    private int codeLength;

    private CodeAttributeEditor codeAttributeEditor;


    /**
     * Resets the accumulated code changes.
     * @param codeLength the length of the code that will be edited next.
     */
    public void reset(int codeLength)
    {
        this.codeLength = codeLength;

        // The code attribute editor has to be created lazily.
        if (codeAttributeEditor != null)
        {
            codeAttributeEditor.reset(codeLength);
        }
    }


    // Implementations for InstructionVisitor.

    public void visitSimpleInstruction(Clazz clazz, Method method, CodeAttribute codeAttribute, int offset, SimpleInstruction simpleInstruction)
    {
        // Try to write out the instruction.
        // Simple instructions should always fit.
        simpleInstruction.write(codeAttribute, offset);
    }


    public void visitConstantInstruction(Clazz clazz, Method method, CodeAttribute codeAttribute, int offset, ConstantInstruction constantInstruction)
    {
        try
        {
            // Try to write out the instruction.
            constantInstruction.write(codeAttribute, offset);
        }
        catch (IllegalArgumentException exception)
        {
            // Create a new constant instruction that will fit.
            Instruction replacementInstruction =
                new ConstantInstruction(constantInstruction.opcode,
                                        constantInstruction.constantIndex,
                                        constantInstruction.constant).shrink();

            replaceInstruction(offset, replacementInstruction);

            // Write out a dummy constant instruction for now.
            constantInstruction.constantIndex = 0;
            constantInstruction.constant      = 0;
            constantInstruction.write(codeAttribute, offset);
        }
    }


    public void visitVariableInstruction(Clazz clazz, Method method, CodeAttribute codeAttribute, int offset, VariableInstruction variableInstruction)
    {
        try
        {
            // Try to write out the instruction.
            variableInstruction.write(codeAttribute, offset);
        }
        catch (IllegalArgumentException exception)
        {
            // Create a new variable instruction that will fit.
            Instruction replacementInstruction =
                new VariableInstruction(variableInstruction.opcode,
                                        variableInstruction.variableIndex,
                                        variableInstruction.constant).shrink();

            replaceInstruction(offset, replacementInstruction);

            // Write out a dummy variable instruction for now.
            variableInstruction.variableIndex = 0;
            variableInstruction.constant      = 0;
            variableInstruction.write(codeAttribute, offset);
        }
    }


    public void visitBranchInstruction(Clazz clazz, Method method, CodeAttribute codeAttribute, int offset, BranchInstruction branchInstruction)
    {
        try
        {
            // Try to write out the instruction.
            branchInstruction.write(codeAttribute, offset);
        }
        catch (IllegalArgumentException exception)
        {
            // Create a new unconditional branch that will fit.
            Instruction replacementInstruction =
                new BranchInstruction(InstructionConstants.OP_GOTO_W,
                                      branchInstruction.branchOffset);

            // Create a new instruction that will fit.
            switch (branchInstruction.opcode)
            {
                default:
                {
                    // Create a new branch instruction that will fit.
                    replacementInstruction =
                        new BranchInstruction(branchInstruction.opcode,
                                              branchInstruction.branchOffset).shrink();

                    break;
                }

                // Some special cases, for which a wide branch doesn't exist.
                case InstructionConstants.OP_IFEQ:
                case InstructionConstants.OP_IFNE:
                case InstructionConstants.OP_IFLT:
                case InstructionConstants.OP_IFGE:
                case InstructionConstants.OP_IFGT:
                case InstructionConstants.OP_IFLE:
                case InstructionConstants.OP_IFICMPEQ:
                case InstructionConstants.OP_IFICMPNE:
                case InstructionConstants.OP_IFICMPLT:
                case InstructionConstants.OP_IFICMPGE:
                case InstructionConstants.OP_IFICMPGT:
                case InstructionConstants.OP_IFICMPLE:
                case InstructionConstants.OP_IFACMPEQ:
                case InstructionConstants.OP_IFACMPNE:
                {
                    // Insert the complementary conditional branch.
                    Instruction complementaryConditionalBranch =
                        new BranchInstruction((byte)(((branchInstruction.opcode+1) ^ 1) - 1),
                                              (1+2) + (1+4));

                    insertBeforeInstruction(offset, complementaryConditionalBranch);

                    // Create a new unconditional branch that will fit.
                    break;
                }

                case InstructionConstants.OP_IFNULL:
                case InstructionConstants.OP_IFNONNULL:
                {
                    // Insert the complementary conditional branch.
                    Instruction complementaryConditionalBranch =
                        new BranchInstruction((byte)(branchInstruction.opcode ^ 1),
                                              (1+2) + (1+4));

                    insertBeforeInstruction(offset, complementaryConditionalBranch);

                    // Create a new unconditional branch that will fit.
                    break;
                }
            }

            replaceInstruction(offset, replacementInstruction);

            // Write out a dummy branch instruction for now.
            branchInstruction.branchOffset = 0;
            branchInstruction.write(codeAttribute, offset);
        }
    }


    public void visitAnySwitchInstruction(Clazz clazz, Method method, CodeAttribute codeAttribute, int offset, SwitchInstruction switchInstruction)
    {
        // Try to write out the instruction.
        // Switch instructions should always fit.
        switchInstruction.write(codeAttribute, offset);
    }


    // Implementations for AttributeVisitor.

    public void visitCodeAttribute(Clazz clazz, Method method, CodeAttribute codeAttribute)
    {
        // Avoid doing any work if nothing is changing anyway.
        if (codeAttributeEditor != null)
        {
            // Apply the collected expansions.
            codeAttributeEditor.visitCodeAttribute(clazz, method, codeAttribute);

            // Clear the modifications for the next run.
            codeAttributeEditor = null;
        }
    }


    // Small utility methods.

    /**
     * Remembers to place the given instruction right before the instruction
     * at the given offset.
     */
    private void insertBeforeInstruction(int instructionOffset, Instruction instruction)
    {
        ensureCodeAttributeEditor();

        // Replace the instruction.
        codeAttributeEditor.insertBeforeInstruction(instructionOffset, instruction);
    }


    /**
     * Remembers to replace the instruction at the given offset by the given
     * instruction.
     */
    private void replaceInstruction(int instructionOffset, Instruction instruction)
    {
        ensureCodeAttributeEditor();

        // Replace the instruction.
        codeAttributeEditor.replaceInstruction(instructionOffset, instruction);
    }


    /**
     * Remembers to place the given instruction right after the instruction
     * at the given offset.
     */
    private void insertAfterInstruction(int instructionOffset, Instruction instruction)
    {
        ensureCodeAttributeEditor();

        // Replace the instruction.
        codeAttributeEditor.insertAfterInstruction(instructionOffset, instruction);
    }


    /**
     * Makes sure there is a code attribute editor for the given code attribute.
     */
    private void ensureCodeAttributeEditor()
    {
        if (codeAttributeEditor == null)
        {
            codeAttributeEditor = new CodeAttributeEditor();
            codeAttributeEditor.reset(codeLength);
        }
    }
}
